# Test Parity

Run parity tests comparing Rust backend output against reference fixtures.

## When to Use

- After modifying node implementations in `kexedit-sim-nodes`
- After changing physics calculations in `kexedit-sim`
- After updating track evaluation in `kexedit-track`
- Before committing changes to ensure no regressions

## Commands

```bash
# Run all tests including parity tests
cargo test

# Run specific crate tests
cargo test -p kexedit-sim
cargo test -p kexedit-sim-nodes
cargo test -p kexedit-track

# Run with output for debugging
cargo test -- --nocapture
```

## Test Fixtures

Located in `test-data/`:
- `all_types.json` - Tests all node types
- `circuit.json` - Complete circuit coaster
- `shuttle.json` - Shuttle coaster with reversals
- `switch.json` - Track switching scenarios

## Interpreting Failures

Parity failures indicate output differs from reference. Check:
1. Was the change intentional? Update fixture if so.
2. Physics constant change? Verify energy model.
3. Node behavior change? Check keyframe evaluation.
