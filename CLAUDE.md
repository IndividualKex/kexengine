# kexengine

Standalone Rust library for Force Vector Design (FVD) simulation and roller coaster track generation.

## Architecture

**Single crate with layered modules** — Strict inward dependencies only.

**Dependency rule**: Lower layers never import from higher layers. Cores are pure and reusable.

```
┌─────────────────────────────────────────────┐
│  ffi (feature-gated)  C FFI adapter         │
├─────────────────────────────────────────────┤
│  track                Evaluation/splines    │
├─────────────────────────────────────────────┤
│  nodes                Node implementations  │
├─────────────────────────────────────────────┤
│  graph                DAG structure         │
├─────────────────────────────────────────────┤
│  sim                  Physics primitives    │
└─────────────────────────────────────────────┘
```

| Module | Purpose | Dependencies |
|--------|---------|--------------|
| `sim` | Pure physics/math: frames, points, quaternions, constants | (none) |
| `graph` | Graph structure, topological sort, port queries | (none) |
| `nodes` | Node types (Force, Geometric, Bridge, etc.) and schema | sim |
| `track` | Document evaluation, sections, arc-length splines | nodes, graph, sim |
| `ffi` | `extern "C"` exports (feature-gated) | track, graph, sim |

## Integration

**Via Rust crate**:
```rust
use kexengine::{Graph, Float3, Frame, Point};
use kexengine::track::{evaluate_graph, build_sections, resample};
use kexengine::nodes::{NodeType, NodeSchema};
use kexengine::sim::physics;
```

**Via C FFI** (enable `ffi` feature):
- Build with `cargo build --release`
- Link the resulting `kexengine.dll`/`libkexengine.so`/`libkexengine.dylib`
- Functions use `kex_` prefix

## FFI API

Single-call design: `kex_build(doc, resolution, default_style, output) → error_code`

- Takes `KexDocument` (graph + inputs + keyframes)
- Fills `KexOutput` (points, sections, traversal, spline data)
- One FFI crossing per build

**Error codes:** 0=success, -1=null, -3=buffer overflow (resize and retry), -4=cycle detected

---

## Commands

```bash
cargo test              # All tests
cargo build --release   # Build library
cargo clippy            # Lint
cargo fmt               # Format
```

---

## Physics

**Energy Model**:
```
E = ½v² + G·y + G·friction·distance
dE = -v³·resistance·dt
v = √(2·(E - PE))
```

**Constants** (`sim::physics`): `G=9.80665`, `HZ=100`, `DT=0.01`, `MIN_VELOCITY=1e-3`

---

## Style

- Self-explanatory code, minimal comments
- snake_case files, PascalCase types, SCREAMING_SNAKE_CASE constants
- FFI: `#[no_mangle] unsafe extern "C"`, null-safe pointers

## Tests

**Inline `#[cfg(test)] mod tests`** — Unit tests in same file as code.

---

## Node Types

| Node | Purpose |
|------|---------|
| Force | Normal/lateral force sections |
| Geometric | Pitch/yaw/roll steering |
| Curved | Arc paths with lead-in/out |
| Bridge | Cubic Bezier connections |
| CopyPath | Matrix-transformed following |
| Anchor | Initial state |
| Reverse | Direction reversal |
| ReversePath | Path order reversal |
