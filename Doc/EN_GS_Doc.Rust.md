# GS.Phy2D - Rust Implementation

*Guide for Pascal/C++ developers discovering Rust - English Version*

---

## 1. Why Rust?

I am a Delphi dev (mastery) and C (mastery), C++ (comfortable, nothing more) and Rust may seem different, but once past the memory specifics, it's quite nice! I even think I prefer it to C++! The syntax is nicer, and the "pitfalls" (IMHO) of C++ are avoided (overloading, template verbosity, etc.)

The **advantages** (Always IMHO)
- Memory safety: no dangling pointers, no buffer overflows (verified at compile time)
- No garbage collector: predictable performance like C/C++, Pascal
- Zero-cost abstractions: generics and traits cost nothing at runtime

---

## 2. Project Structure

```
Solutions/Rust/
├── Cargo.toml          ← Project configuration
├── src/
│   ├── lib.rs          ← Library entry point
│   ├── gs_phy_vec2.rs  ← Vector mathematics
│   ├── gs_phy_types.rs ← Base types
│   ├── gs_phy_aabb.rs  ← Dynamic AABBs
│   ├── gs_phy_spatial_hash.rs ← Spatial partitioning
│   ├── gs_phy_renderer.rs     ← Colors and rendering
│   ├── gs_phy_world.rs        ← Main engine
│   └── bin/
│       └── gs_phy2d_demo.rs   ← Demo application
```

### Cargo.toml

```toml
[package]
name = "gs_phy2d"
version = "0.1.0"
edition = "2021"

[dependencies]
macroquad = "0.4"  # For rendering and inputs
rand = "0.8"       # For random positions
```

---

## 3. Base Types

### Vec2 - Rust Equivalent of TVec2

```rust
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}
```

**The derives** (Delphi equivalent):
- `Clone`: allows `.clone()` (explicit copy)
- `Copy`: implicit copy (like an `Integer` in Delphi)
- `Debug`: allows `println!("{:?}", vec)` (debug)
- `Default`: provides `Vec2::default()` which gives `(0, 0)`
- `PartialEq`: allows `==` and `!=`

### Operator Implementation

In Rust, traits are implemented to overload operators:

```rust
impl Add for Vec2 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}
```

Now `vec_a + vec_b` works. It's like `operator+` in C++ or class operators in modern Delphi.

---

## 4. The Engine: PhyWorld

### Declaration

```rust
pub struct PhyWorld {
    particles: ParticleSoA,      // SoA format
    aabbs: AABBSoA,
    boxes: Vec<StaticBox>,       // Vec<T> = dynamic array
    constraints: Vec<Constraint>,
    rigid_bodies: Vec<RigidBody>,
    spatial_hash: SpatialHash,
    box_grid: BoxGrid,

    gravity: Vec2,
    damping: f32,
    world_width: f32,
    world_height: f32,
    collision_iterations: usize,  // usize = unsigned integer
    restitution: f32,
}
```

**Differences with Delphi**:
- No `private`/`public` in the struct, it's separated via `pub`
- `Vec<T>` = dynamic array (like `TArray<T>` or `array of T`)
- `usize` = integer the size of a pointer (32 or 64 bits depending on platform)

### step() Method

```rust
pub fn step(&mut self, dt: f32) {
    // 1. Verlet integration
    self.integrate(dt);
    self.integrate_aabbs(dt);

    // 2. Collision iterations
    for _ in 0..self.collision_iterations {
        self.solve_constraints();
        self.solve_collisions();
        self.solve_box_collisions();
        self.solve_aabb_box_collisions();
    }
}
```

**The `&mut self`**: in Rust, you must explicitly state if you modify the object. `&self` = read only, `&mut self` = modification.

---

## 5. SoA in Rust

### ParticleSoA

```rust
#[derive(Default)]
pub struct ParticleSoA {
    pub pos_x: Vec<f32>,
    pub pos_y: Vec<f32>,
    pub old_pos_x: Vec<f32>,
    pub old_pos_y: Vec<f32>,
    pub accel_x: Vec<f32>,
    pub accel_y: Vec<f32>,
    pub radius: Vec<f32>,
    pub inv_mass: Vec<f32>,
    pub restitution: Vec<f32>,
    pub flags: Vec<u8>,
    pub body_id: Vec<i32>,  // -1 = free
}
```

Identical to the Delphi concept but with Rust syntax.

### Data Access

```rust
// Delphi: FParticles.PosX[I]
// Rust:
self.particles.pos_x[i]
```

---

## 6. Verlet Integration

```rust
fn integrate(&mut self, dt: f32) {
    let dt2 = dt * dt;
    let grav_x = self.gravity.x;
    let grav_y = self.gravity.y;
    let damping = self.damping;

    for i in 0..self.particles.len() {
        // Skip fixed particles
        if (self.particles.flags[i] & PHY_FLAG_FIXED) != 0 {
            continue;
        }

        // Verlet
        let vel_x = (self.particles.pos_x[i] - self.particles.old_pos_x[i]) * damping;
        let vel_y = (self.particles.pos_y[i] - self.particles.old_pos_y[i]) * damping;

        self.particles.accel_x[i] += grav_x;
        self.particles.accel_y[i] += grav_y;

        let new_pos_x = self.particles.pos_x[i] + vel_x + self.particles.accel_x[i] * dt2;
        let new_pos_y = self.particles.pos_y[i] + vel_y + self.particles.accel_y[i] * dt2;

        self.particles.old_pos_x[i] = self.particles.pos_x[i];
        self.particles.old_pos_y[i] = self.particles.pos_y[i];
        self.particles.pos_x[i] = new_pos_x;
        self.particles.pos_y[i] = new_pos_y;

        self.particles.accel_x[i] = 0.0;
        self.particles.accel_y[i] = 0.0;
    }
}
```

**Observation**: the code is almost identical to Delphi. Rust does not add complexity for this kind of numerical code.

---

## 7. FastInvSqrt in Rust

```rust
#[inline]
pub fn fast_inv_sqrt(x: f32) -> f32 {
    let x2 = x * 0.5;

    // unsafe: bit reinterpretation (type punning)
    let i = unsafe { std::mem::transmute::<f32, u32>(x) };
    let i = 0x5F3759DF - (i >> 1);
    let y = unsafe { std::mem::transmute::<u32, f32>(i) };

    // Newton-Raphson
    y * (1.5 - x2 * y * y)
}
```

**The `unsafe` keyword**: Rust forbids by default operations that could cause memory bugs. Here, the bit cast `f32 → u32` requires `unsafe` because Rust cannot verify that it's intentional. It is, I believe, safe in this context.

---

## 8. Collision with BodyID

```rust
fn collide_particles(&mut self, i: usize, j: usize) {
    // Skip if same rigid body
    let body_i = self.particles.body_id[i];
    let body_j = self.particles.body_id[j];
    if body_i >= 0 && body_i == body_j {
        return;  // Same body = no intra-body collision
    }

    // ... rest of collision
}
```

Identical to Delphi logic.

---

## 9. Rust Particularities

### The Borrow Checker

This is THE destabilizing thing!!
Example of code that doesn't compile:

```rust
fn solve_box_collisions(&mut self) {
    // ❌ DOES NOT COMPILE:
    if let Some(cell) = self.box_grid.get_cell(x, y) {
        for k in 0..cell.count {
            // Here we borrow self.box_grid for reading
            self.collide_particle_static_box(i, cell.box_indices[k]);
            // And there we want to borrow self for writing → CONFLICT
        }
    }
}
```

**Solution**: copy data locally before modifying:

```rust
fn solve_box_collisions(&mut self) {
    if let Some(cell) = self.box_grid.get_cell(pos_x, pos_y) {
        // Copy indices into a local buffer
        let count = cell.count;
        let mut box_indices = [0usize; 8];
        for k in 0..count {
            box_indices[k] = cell.box_indices[k];
        }

        // Now we can modify self freely
        for k in 0..count {
            self.collide_particle_static_box(i, box_indices[k]);
        }
    }
}
```

It's a bit verbose but it prevents memory bugs. Better than that, it sometimes pushes to rethink the high-level architecture to avoid copies.


### Closures

```rust
// Delphi: for I := 0 to Count - 1 do
// Rust (functional):
(0..count).for_each(|i| {
    // ...
});

// But the classic loop also works:
for i in 0..count {
    // ...
}
```

---

## 10. Compilation and Execution

```bash
# Compile the library
cargo build --release

# Run the demo
cargo run --release --bin gs_phy2d_demo

# Unit tests
cargo test
```

The `--release` option enables optimizations. In debug, Rust is quite slow (bounds checking everywhere).

---

## 11. The Demo with Macroquad

```rust
#[macroquad::main(window_conf)]
async fn main() {
    let mut world = PhyWorld::new();
    world.set_gravity(0.0, 800.0);

    loop {
        // Input
        if is_key_pressed(KeyCode::B) {
            spawn_balls(&mut world, 50);
        }

        // Simulation
        world.step(1.0 / 60.0);

        // Rendering
        clear_background(WHITE);
        render_world(&world);

        next_frame().await;  // ← async/await for render loop
    }
}
```

**The `async/await`**: Macroquad uses async to manage the render loop. It may be surprising but it's just a library convention.

---

## Tips for Pascal Developers

1. **`let` vs `let mut`**: by default variables are immutable. Use `let mut` to be able to modify.

2. **No nil/null**: use `Option<T>` (equivalent of `Maybe` or nullable).

3. **No exceptions**: use `Result<T, E>` for errors.

4. **The `;` is mandatory** except for the last expression of a block (which becomes the return value).

5. **`::` vs `.`**: `::` for associated functions (static), `.` for methods.

6. **Cargo > IDE**: the Rust workflow goes through the terminal. Cargo does everything: build, test, docs, dependencies.

---

## Conclusion

The GS.Phy2D port to Rust keeps the same structure and the same algorithms as the Delphi version. The differences are mainly syntactic and related to the borrow checker.

For an experienced Pascal developer, the Rust learning curve is reasonable. The hardest part is not the syntax but the mindset: accepting that the compiler refuses code that "should" work. Once past this stage, the code benefits from the guarantee that if it compiles, there will be no crash related to invalid memory access, well, that's the promise ;) we shall see!
