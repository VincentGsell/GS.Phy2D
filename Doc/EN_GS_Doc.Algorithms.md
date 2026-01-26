# GS.Phy2D - Algorithms and Mathematics

*Technical Guide - English Version*

---

## 1. Verlet Integration

### Why Verlet Rather Than Euler?

Explicit Euler integration (`pos += vel * dt`) becomes unstable beyond a certain number of objects or when forces increase. In game physics, this translates to objects that "explode" or pass through walls.

Verlet integration:

```
new_pos = pos + (pos - old_pos) * damping + accel * dt²
```

Velocity is not stored explicitly. It is *implicit* in the difference `pos - old_pos`. This has the effect of:

1. Avoiding velocity error accumulation
2. Allowing direct modification of `pos` without breaking velocity (useful for constraints)
3. Reducing sensitivity to large forces

### Damping Application

Damping simulates air friction:

```
vel = (pos - old_pos) * damping  // damping = 0.99 typically
```

A damping of 0.99 = 1% velocity loss per frame. This prevents infinite oscillations.

---

## 2. Broad Phase Collision Detection: Spatial Hash

### The O(n²) Problem

Comparing each object with all others = O(n²). With 1000 objects: 1 million tests per frame.

### The Solution: Spatial Hashing

Space is divided into cells. Each object is inserted into the cell(s) it overlaps. To find potential collisions, only neighboring cells are examined.

```
┌───┬───┬───┬───┐
│   │ ● │   │   │   ← Only objects in the same
├───┼───┼───┼───┤     cells are compared
│   │   │ ● │ ● │
├───┼───┼───┼───┤
│ ● │   │   │   │
└───┴───┴───┴───┘
```

Average complexity: O(n) instead of O(n²).

### Optimal Cell Size

Recommended size: ~2× the average radius of objects. Too small = objects in too many cells. Too large = too many objects per cell.

### The Higher Tag Trick

To avoid testing pair (A,B) then (B,A), tag encoding is used. During a query, only objects with a tag **higher** than the source object's tag are returned. Each pair is tested only once.

---

## 3. Narrow Phase: Collision Tests

### Circle vs Circle

```
dist² = (x2-x1)² + (y2-y1)²
collision = dist² < (r1 + r2)²
```

Squares are compared to avoid the square root.

### Circle vs AABB

1. Find the closest point on the AABB to the circle:
   ```
   closest.x = clamp(circle.x, aabb.min_x, aabb.max_x)
   closest.y = clamp(circle.y, aabb.min_y, aabb.max_y)
   ```

2. Test the circle-point distance:
   ```
   dist² = (circle.x - closest.x)² + (circle.y - closest.y)²
   collision = dist² < radius²
   ```

### AABB vs AABB

Separation test on both axes:

```
collision = !(a.max_x < b.min_x || a.min_x > b.max_x ||
              a.max_y < b.min_y || a.min_y > b.max_y)
```

---

## 4. Collision Resolution

### Separation

When two objects overlap, the overlap and collision normal are calculated, then separation is applied proportionally to inverse masses:

```
overlap = min_dist - distance
ratio1 = inv_mass1 / (inv_mass1 + inv_mass2)
ratio2 = inv_mass2 / (inv_mass1 + inv_mass2)

pos1 -= normal * overlap * ratio1
pos2 += normal * overlap * ratio2
```

A heavy object (small inv_mass) moves less than a light object.

### Impulse (Bounce)

After separation, the relative velocity along the normal is calculated:

```
v_rel = dot(v1 - v2, normal)
```

If `v_rel > 0`, objects are already moving apart: nothing to do.

Otherwise, an impulse is applied:

```
j = -(1 + restitution) * v_rel / (inv_mass1 + inv_mass2)
```

In Verlet, instead of modifying an explicit velocity, `old_pos` is modified:

```
old_pos1 = pos1 - (vel1 - impulse * inv_mass1)
old_pos2 = pos2 - (vel2 + impulse * inv_mass2)
```

---

## 5. Distance Constraints

### Principle

Two connected particles must remain at a fixed distance (`rest_length`). If they move apart or closer, they are corrected:

```
delta = p2 - p1
distance = length(delta)
error = distance - rest_length
correction = delta * (error / distance) * 0.5 * stiffness
```

`correction` is applied to p1 and `-correction` to p2 (proportionally to masses).

### Stiffness

- `1.0`: complete correction = rigid
- `0.5`: partial correction = semi-elastic
- `0.1`: weak correction = soft spring

### Rigid Bodies

A rigid rectangle = 4 particles at corners + 6 constraints:
- 4 sides (maintain the perimeter)
- 2 diagonals (prevent deformation into a parallelogram)

```
P0 ─────── P1
│  \   /   │
│   \ /    │
│    X     │   ← Diagonals = rigidity
│   / \    │
│  /   \   │
P3 ─────── P2
```

---

## 6. FastInvSqrt - Fast Inverse Square Root

### Algorithm Known from Video Games

Used in Quake III (1999):

```c
float fast_inv_sqrt(float x) {
    float x2 = x * 0.5f;
    int i = *(int*)&x;              // Bit hack
    i = 0x5F3759DF - (i >> 1);      // Constant
    float y = *(float*)&i;
    return y * (1.5f - x2 * y * y); // Newton-Raphson
}
```

### Usage

Vector normalization requires `1/length = 1/sqrt(x² + y²)`. With FastInvSqrt, two costly operations (sqrt and division) are avoided.

Note: on recent CPUs with SSE/AVX, the `rsqrtss` instruction does the same work in hardware.

---

## 7. Tag Encoding

To identify the object type in the spatial hash, type + index are encoded in a single integer:

```
[3 bits: type][61 bits: index]
```

- Bits 63-61: type (0 = particle, 1 = AABB, etc.)
- Bits 60-0: index in the array

Functions:
```
encode(type, index) = (type << 61) | index
decode(tag) → (type, index)
```

Avoids polymorphic structures or enums with dynamic dispatch.

---

## 8. Structure of Arrays (SoA)

### Array of Structures (AoS) - Classic

```
particles[i].pos_x
particles[i].pos_y
particles[i].radius
```

In memory: `[pos_x, pos_y, radius, pos_x, pos_y, radius, ...]`

### Structure of Arrays (SoA)

```
pos_x[i]
pos_y[i]
radius[i]
```

In memory: `[pos_x, pos_x, pos_x, ...][pos_y, pos_y, ...][radius, radius, ...]`

### Why SoA?

1. **Cache locality**: when iterating over `pos_x`, all values are contiguous
2. **SIMD**: the CPU can load 4/8 values at once
3. **Prefetch**: the CPU better predicts memory accesses

Measured gain: 2-3× on dense simulations.

---

## References

- [Verlet Integration](https://en.wikipedia.org/wiki/Verlet_integration) - Wikipedia
- [Fast Inverse Square Root](https://en.wikipedia.org/wiki/Fast_inverse_square_root) - Wikipedia
- [Game Physics Engine Development](https://www.amazon.com/Game-Physics-Engine-Development-Commercial-Grade/dp/0123819768) - Ian Millington
- [Advanced Character Physics](https://www.cs.cmu.edu/afs/cs/academic/class/15462-s13/www/lec_slides/Jakobsen.pdf) - Thomas Jakobsen (GDC 2001)
