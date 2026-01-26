# GS.Phy2D - Delphi vs Rust Comparison

*Comparative Analysis of the Two Implementations - English Version*

---

## 1. Overview

| Aspect | Delphi | Rust |
|--------|--------|------|
| Paradigm | OOP + procedural | Multi-paradigm (functional + imperative) |
| Memory management | Manual + ARC (interfaces) | Ownership + Borrowing |
| Compilation | Fast | Slow (optimized code via LLVM) |
| Ecosystem | Mature, niche | Growing, modern |
| Portability | Windows (+ mobile/Linux) | All platforms |
| Learning curve | Gentle | Steep (borrow checker) |

---

## 2. Syntax Comparison

### Structure Declaration

```pascal
// Delphi
type
  TParticle = record
    PosX, PosY: Single;
    Radius: Single;
    InvMass: Single;
  end;
```

```rust
// Rust
pub struct Particle {
    pub pos_x: f32,
    pub pos_y: f32,
    pub radius: f32,
    pub inv_mass: f32,
}
```

**Observation**: very similar. Rust uses `snake_case`, Delphi `PascalCase`.

### Iteration Loop

```pascal
// Delphi
for I := 0 to Count - 1 do
begin
  if Condition then
    Continue;
  DoSomething(I);
end;
```

```rust
// Rust
for i in 0..count {
    if condition {
        continue;
    }
    do_something(i);
}
```

**Observation**: Rust syntax is more concise. `0..count` is equivalent to `[0, count[`.

### Dynamic Array Creation

```pascal
// Delphi
var
  Arr: array of Single;
begin
  SetLength(Arr, 100);
  Arr[0] := 1.0;
end;
```

```rust
// Rust
let mut arr: Vec<f32> = Vec::with_capacity(100);
arr.push(1.0);
// or
let mut arr = vec![0.0; 100];  // 100 elements at 0.0
arr[0] = 1.0;
```

### Function with Return

```pascal
// Delphi
function FastInvSqrt(X: Single): Single;
begin
  // ...
  Result := Y * (1.5 - X2 * Y * Y);
end;
```

```rust
// Rust
fn fast_inv_sqrt(x: f32) -> f32 {
    // ...
    y * (1.5 - x2 * y * y)  // No "return" if last expression. Quite "cool" :)
}
```

---

## 3. Memory Management

### Delphi

```pascal
// Records: manual memory management (stack or contiguous block)
var
  P: TParticle;  // Allocated on stack

// Classes: Create/Free or ARC (interfaces)
var
  World: TPhyWorld;
begin
  World := TPhyWorld.Create;
  try
    // ...
  finally
    World.Free;
  end;
end;
```

### Rust

```rust
// Structs: automatic ownership
let p = Particle::new(...);  // "Owned" by variable p
// p is freed automatically when it goes out of scope

// References: explicit borrowing
let world = PhyWorld::new();
process(&world);      // Borrows for reading
modify(&mut world);   // Borrows for writing
```

**The borrow checker**: Rust verifies at compile time that you cannot have simultaneously a mutable reference and other references. This prevents data races and use-after-free.

---

## 4. SoA Pattern

Both implementations use Structure of Arrays:

```pascal
// Delphi
TParticleSoA = record
  PosX: array of Single;
  PosY: array of Single;
  // ...
end;
```

```rust
// Rust
struct ParticleSoA {
    pos_x: Vec<f32>,
    pos_y: Vec<f32>,
    // ...
}
```

**Conceptually identical**, just different syntax.

---

## 5. Spatial Hash - Subtle Differences

### Delphi

```pascal
// Cells = flattened 2D array
FCells: array of array of Cardinal;  // Nested dynamic arrays

procedure TSpatialHash.Insert(Tag: Cardinal; X, Y, Radius: Single);
begin
  // Direct cell access
  FCells[CellIdx][FCellCounts[CellIdx]] := Tag;
  Inc(FCellCounts[CellIdx]);
end;
```

### Rust

```rust
// Cells = Vec of Vec
cells: Vec<Vec<usize>>,

fn insert(&mut self, tag: usize, x: f32, y: f32, radius: f32) {
    // push adds at the end
    self.cells[cell_idx].push(tag);
    self.cell_counts[cell_idx] += 1;
}
```

**Difference**: Rust uses `.push()` which automatically handles resizing. In Delphi, you must pre-allocate or use `SetLength`.

---

## 6. Error Handling

### Delphi

```pascal
try
  DoRiskyThing;
except
  on E: Exception do
    ShowMessage(E.Message);
end;
```

### Rust

```rust
// Option for optional values
fn get_cell(&self, x: f32, y: f32) -> Option<&BoxGridCell> {
    if in_bounds {
        Some(&self.cells[idx])
    } else {
        None
    }
}

// Usage
if let Some(cell) = grid.get_cell(x, y) {
    // cell exists
} else {
    // out of bounds
}
```

**Different philosophy**: Delphi uses exceptions (runtime), Rust encodes errors in types (compile-time).

---

## 7. Performance

### Compilation

| | Delphi | Rust |
|---|---|---|
| Incremental build | ~1-2s | ~3-10s |
| Full build | ~5-10s | ~30-60s |
| Generated code | Good | Excellent |

Rust compiles more slowly but produces more optimized code thanks to LLVM.

### Execution

- **Delphi**: optional bounds checking, aggressive inlining
- **Rust**: bounds checking by default (disableable), better SIMD auto-vectorization

For GS.Phy2D, performance is better in Rust (referring to FPS, we're close to a factor of 5/6)
But it must be said that Delphi doesn't benefit from as fast a graphics backend. I will do a port to a 3D backend when I have the opportunity.

---

## 8. What is Simpler

### In Delphi

- **FMX rendering**: integrated, drag & drop interface
- **Rapid prototyping**: very short compile/run cycle
- **FMX interop**: Native multiplatform.
- **Integrated IDE**: debugger, profiler, designer

### In Rust

- **Dependency management**: `cargo add macroquad` and it's done
- **Documentation**: `cargo doc --open` generates HTML docs
- **Tests**: `#[test]` built-in, no external framework needed
- **Cross-compilation**: one command to target Linux from Windows
- **Memory guarantees**: no memory leaks, no data races

---

## 9. What is More Complicated

### In Delphi

- **Memory management**: Manual. (no interfaces here, I might add some)
- **Multithreading**: Somewhat dated approach. I avoided generics, targeting performance. Race condition threat always present.
- **Licenses**: non-negligible cost

### In Rust

- **Borrow checker**: the compiler refuses conceptually valid code
- **Lifetime annotations**: `fn foo<'a>(x: &'a str) -> &'a str`
- **Async**: Rust's async model is powerful but complex
- **Gaming ecosystem**: less mature than Unity/Unreal/Delphi

---

## 10. Identical vs Different Code

### Identical (~80%)

- Verlet integration
- Collision tests (circle-circle, AABB-AABB)
- Constraint resolution
- Impulse calculations
- General World structure

### Different (~20%)

- Borrow management (local index copy in solve_box_collisions)
- Tag encoding (32 bits in Delphi, 64 bits in Rust)
- Cell management in spatial hash (pre-allocation vs push)
- Public API (explicit getters in Rust)

---

## 11. Verdict

### Delphi if...

- You mainly target Windows
- You need a native graphical interface quickly
- You're working on an existing Delphi project
- Compilation time matters for your workflow
- You prefer an all-in-one IDE

### Rust if...

- You want memory safety guarantees
- You target multiple platforms (Linux, WASM, embedded)
- You want a modern package ecosystem
- You accept a steeper learning curve
- You prefer command line and modern editors (VS Code)

### For GS.Phy2D specifically

- Both versions work in an equivalent manner.
- With increased performance on the Rust side, but that's a compiler story.
- I think I'll keep these projects in both languages for the future, even though Rust seems to have a bright future, at least at the time I write these lines.

---

## Conclusion

The GS.Phy2D port shows that physics algorithms are **language-independent**. The real difference is in the **memory management philosophy** and the **ecosystem**.

Delphi: "do what I say".
Rust: "prove to me it's safe". ;)
