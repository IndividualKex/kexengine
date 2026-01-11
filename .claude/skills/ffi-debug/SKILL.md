# FFI Debug

Debug issues with the C FFI interface in `kexedit-ffi`.

## When to Use

- FFI calls returning unexpected error codes
- Memory layout mismatches between Rust and C
- Null pointer or buffer sizing issues
- Integration problems with external consumers

## Error Codes

The FFI functions return status codes:
- `0` - Success
- `-1` - Null pointer passed (check all pointer arguments)
- `-2` - Invalid enum/type value (e.g., invalid duration_type)
- `-3` - Output buffer overflow (pre-allocate larger buffers)
- `-4` - Graph cycle detected (check graph connectivity)

## kexedit_track_build

Main entry point. Signature:
```c
int kexedit_track_build(
    const DocumentData* doc,
    float resolution,
    int default_style_index,
    TrackOutput* out
);
```

Caller must pre-allocate all output buffers in `TrackOutput`. On `-3`, increase buffer sizes and retry.

## Debugging Commands

```bash
# Build debug FFI for better stack traces
cargo build -p kexedit-ffi

# Run FFI tests with output
cargo test -p kexedit-ffi -- --nocapture
```

## Memory Layout

All FFI structs use `#[repr(C)]`. Verify struct definitions match between Rust and consumer. Check `kexedit-ffi/src/lib.rs` for canonical definitions.
