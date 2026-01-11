//! Single-call FFI for kexengine.
//!
//! One function: `kex_build` - takes document, produces track.
//!
//! # Error Codes
//! - `0`: Success
//! - `-1`: Null pointer
//! - `-3`: Buffer overflow (resize and retry)
//! - `-4`: Cycle detected

use crate::graph::Graph;
use crate::sim::{Float3, Keyframe, Point};
use crate::track::{
    build_sections, build_traversal_order, collect_sections, compute_continuations,
    compute_spatial_continuations, evaluate_graph, interpolate_physics, resample, DocumentView,
    Section, SplinePoint,
};
use std::collections::HashMap;

/// All document data needed to build track.
#[repr(C)]
pub struct KexDocument {
    // Graph arrays - nodes
    pub node_ids: *const u32,
    pub node_count: usize,
    pub node_types: *const u32,
    pub node_input_counts: *const i32,
    pub node_output_counts: *const i32,

    // Graph arrays - ports
    pub port_ids: *const u32,
    pub port_count: usize,
    pub port_types: *const u32,
    pub port_owners: *const u32,
    pub port_is_input: *const u8,

    // Graph arrays - edges
    pub edge_ids: *const u32,
    pub edge_count: usize,
    pub edge_sources: *const u32,
    pub edge_targets: *const u32,

    // Property maps - scalars
    pub scalar_keys: *const u64,
    pub scalar_values: *const f32,
    pub scalar_count: usize,

    // Property maps - vectors
    pub vector_keys: *const u64,
    pub vector_values: *const Float3,
    pub vector_count: usize,

    // Property maps - flags
    pub flag_keys: *const u64,
    pub flag_values: *const i32,
    pub flag_count: usize,

    // Keyframes
    pub keyframes: *const Keyframe,
    pub keyframe_count: usize,
    pub keyframe_range_keys: *const u64,
    pub keyframe_range_starts: *const i32,
    pub keyframe_range_lengths: *const i32,
    pub keyframe_range_count: usize,
}

/// Output buffers for track data.
#[repr(C)]
pub struct KexOutput {
    // Raw simulation points
    pub points: *mut Point,
    pub points_capacity: usize,

    // Sections
    pub sections: *mut Section,
    pub sections_capacity: usize,
    pub section_node_ids: *mut u32,

    // Traversal order
    pub traversal_order: *mut i32,
    pub traversal_capacity: usize,

    // Spline data (resampled)
    pub spline_points: *mut SplinePoint,
    pub spline_capacity: usize,
    pub spline_velocities: *mut f32,
    pub spline_normal_forces: *mut f32,
    pub spline_lateral_forces: *mut f32,
    pub spline_roll_speeds: *mut f32,

    // Output counts (written by kex_build)
    pub points_count: *mut usize,
    pub sections_count: *mut usize,
    pub traversal_count: *mut usize,
    pub spline_count: *mut usize,
}

/// Build track from document data.
///
/// Single FFI call: document in → track out.
///
/// # Safety
///
/// - `doc` must be a valid pointer to a properly initialized `KexDocument`
/// - `output` must be a valid pointer to a `KexOutput` with sufficient capacity
/// - All array pointers in `doc` must be valid for their respective counts
/// - Output buffer pointers must be valid and have capacity >= their `*_capacity` fields
#[no_mangle]
pub unsafe extern "C" fn kex_build(
    doc: *const KexDocument,
    resolution: f32,
    default_style_index: i32,
    output: *mut KexOutput,
) -> i32 {
    if doc.is_null() || output.is_null() {
        return -1;
    }
    let doc = &*doc;
    let output = &mut *output;

    // Build graph
    let port_is_input: Vec<bool> = if doc.port_count > 0 && !doc.port_is_input.is_null() {
        std::slice::from_raw_parts(doc.port_is_input, doc.port_count)
            .iter()
            .map(|&b| b != 0)
            .collect()
    } else {
        Vec::new()
    };

    let graph = if doc.node_count > 0 {
        Graph::from_vecs(
            to_vec(doc.node_ids, doc.node_count),
            to_vec(doc.node_types, doc.node_count),
            to_vec(doc.node_input_counts, doc.node_count),
            to_vec(doc.node_output_counts, doc.node_count),
            to_vec(doc.port_ids, doc.port_count),
            to_vec(doc.port_types, doc.port_count),
            to_vec(doc.port_owners, doc.port_count),
            port_is_input,
            to_vec(doc.edge_ids, doc.edge_count),
            to_vec(doc.edge_sources, doc.edge_count),
            to_vec(doc.edge_targets, doc.edge_count),
        )
    } else {
        Graph::new()
    };

    // Build input maps
    let scalars = to_map(doc.scalar_keys, doc.scalar_values, doc.scalar_count);
    let vectors = to_map(doc.vector_keys, doc.vector_values, doc.vector_count);
    let flags = to_map(doc.flag_keys, doc.flag_values, doc.flag_count);

    // Build keyframes
    let keyframes: Vec<Keyframe> = to_vec(doc.keyframes, doc.keyframe_count);
    let keyframe_ranges = to_keyframe_ranges(
        doc.keyframe_range_keys,
        doc.keyframe_range_starts,
        doc.keyframe_range_lengths,
        doc.keyframe_range_count,
    );

    // Create document view
    let doc_view = DocumentView::new(
        &graph,
        &scalars,
        &vectors,
        &flags,
        &keyframes,
        &keyframe_ranges,
    );

    // Evaluate graph (topological sort + node evaluation)
    let eval_result = match evaluate_graph(&doc_view) {
        Some(r) => r,
        None => return -4,
    };

    // Get topologically sorted node list for section building
    let sorted = match graph.topological_sort() {
        Some(s) => s,
        None => return -4,
    };

    // Build sections
    let (section_nodes, node_to_section) = collect_sections(&sorted, &graph, &eval_result.paths);
    let (points, mut sections) = build_sections(
        &section_nodes,
        &eval_result.paths,
        &doc_view,
        default_style_index as u8,
    );
    compute_continuations(&section_nodes, &node_to_section, &mut sections, &doc_view);
    let traversal = build_traversal_order(&section_nodes, &sections, &doc_view);
    compute_spatial_continuations(&points, &mut sections, &traversal);

    // Check capacities
    if points.len() > output.points_capacity
        || sections.len() > output.sections_capacity
        || traversal.len() > output.traversal_capacity
    {
        return -3;
    }

    // Copy points
    for (i, point) in points.iter().enumerate() {
        *output.points.add(i) = *point;
    }

    // Generate spline data and update section spline indices
    let mut spline_offset = 0usize;
    for section in sections.iter_mut() {
        if !section.is_valid() {
            continue;
        }

        let start = section.start_index as usize;
        let end = section.end_index as usize;
        let path_slice = &points[start..=end];

        let spline = resample(path_slice, resolution);

        if spline_offset + spline.len() > output.spline_capacity {
            return -3;
        }

        section.spline_start_index = spline_offset as i32;
        section.spline_end_index = (spline_offset + spline.len() - 1) as i32;

        for (j, sp) in spline.iter().enumerate() {
            *output.spline_points.add(spline_offset + j) = *sp;

            let (vel, nf, lf, rs) = interpolate_physics(path_slice, sp.arc);
            *output.spline_velocities.add(spline_offset + j) = vel;
            *output.spline_normal_forces.add(spline_offset + j) = nf;
            *output.spline_lateral_forces.add(spline_offset + j) = lf;
            *output.spline_roll_speeds.add(spline_offset + j) = rs;
        }

        spline_offset += spline.len();
    }

    // Copy sections and node IDs
    for (i, section) in sections.iter().enumerate() {
        *output.sections.add(i) = *section;
        *output.section_node_ids.add(i) = section_nodes[i];
    }

    // Copy traversal
    for (i, &idx) in traversal.iter().enumerate() {
        *output.traversal_order.add(i) = idx;
    }

    // Write counts
    *output.points_count = points.len();
    *output.sections_count = sections.len();
    *output.traversal_count = traversal.len();
    *output.spline_count = spline_offset;

    0
}

// --- Helpers ---

unsafe fn to_vec<T: Copy>(ptr: *const T, len: usize) -> Vec<T> {
    if len == 0 || ptr.is_null() {
        Vec::new()
    } else {
        std::slice::from_raw_parts(ptr, len).to_vec()
    }
}

unsafe fn to_map<K: Copy + Eq + std::hash::Hash, V: Copy>(
    keys: *const K,
    values: *const V,
    count: usize,
) -> HashMap<K, V> {
    if count == 0 || keys.is_null() || values.is_null() {
        return HashMap::new();
    }
    std::slice::from_raw_parts(keys, count)
        .iter()
        .copied()
        .zip(std::slice::from_raw_parts(values, count).iter().copied())
        .collect()
}

unsafe fn to_keyframe_ranges(
    keys: *const u64,
    starts: *const i32,
    lengths: *const i32,
    count: usize,
) -> HashMap<u64, (usize, usize)> {
    if count == 0 || keys.is_null() || starts.is_null() || lengths.is_null() {
        return HashMap::new();
    }
    let keys = std::slice::from_raw_parts(keys, count);
    let starts = std::slice::from_raw_parts(starts, count);
    let lengths = std::slice::from_raw_parts(lengths, count);

    (0..count)
        .map(|i| (keys[i], (starts[i] as usize, lengths[i] as usize)))
        .collect()
}
