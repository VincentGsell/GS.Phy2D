# GS.Phy2D - Implémentation Rust

*Guide pour développeurs Pascal/C++ découvrant Rust - Version française*

---

## 1. Pourquoi Rust ?

Je suis dev Delphi (maîtrise) et C (maîtrise), C++ (à l'aise, sans plus) et Rust peut sembler différent, mais une fois passées les spécificités sur la mémoire, c'est plutôt bien ! Je pense même préférer cela à C++ ! La syntaxe est plus sympa, et les "travers" (AMHA) du C++ évités (surcharge, verbosité template, etc.)

Les **avantages** (Tjrs AHMA)
- Sécurité mémoire : pas de dangling pointers, pas de buffer overflows (vérifié à la compilation)
- Pas de garbage collector : performances prédictibles comme C/C++, pascal
- Abstractions zero-cost : les génériques et traits ne coûtent rien à l'exécution

---

## 2. Structure du Projet

```
Solutions/Rust/
├── Cargo.toml          ← Configuration du projet
├── src/
│   ├── lib.rs          ← Point d'entrée de la bibliothèque
│   ├── gs_phy_vec2.rs  ← Mathématiques vectorielles
│   ├── gs_phy_types.rs ← Types de base
│   ├── gs_phy_aabb.rs  ← AABB dynamiques
│   ├── gs_phy_spatial_hash.rs ← Partitionnement spatial
│   ├── gs_phy_renderer.rs     ← Couleurs et rendu
│   ├── gs_phy_world.rs        ← Moteur principal
│   └── bin/
│       └── gs_phy2d_demo.rs   ← Application de démonstration
```

### Cargo.toml

```toml
[package]
name = "gs_phy2d"
version = "0.1.0"
edition = "2021"

[dependencies]
macroquad = "0.4"  # Pour le rendu et les inputs
rand = "0.8"       # Pour les positions aléatoires
```

---

## 3. Types de Base

### Vec2 - Équivalent Rust de TVec2

```rust
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}
```

**Les derives** (équivalent Delphi) :
- `Clone` : permet `.clone()` (copie explicite)
- `Copy` : copie implicite (comme un `Integer` en Delphi)
- `Debug` : permet `println!("{:?}", vec)` (debug)
- `Default` : fournit `Vec2::default()` qui donne `(0, 0)`
- `PartialEq` : permet `==` et `!=`

### Implémentation des opérateurs

En Rust, on implémente des traits pour surcharger les opérateurs :

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

Maintenant `vec_a + vec_b` fonctionne. C'est comme `operator+` en C++ ou les class operators en Delphi moderne.

---

## 4. Le Moteur : PhyWorld

### Déclaration

```rust
pub struct PhyWorld {
    particles: ParticleSoA,      // Format SoA
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
    collision_iterations: usize,  // usize = entier non signé
    restitution: f32,
}
```

**Différences avec Delphi** :
- Pas de `private`/`public` dans la struct, c'est séparé via `pub`
- `Vec<T>` = tableau dynamique (comme `TArray<T>` ou `array of T`)
- `usize` = entier de la taille d'un pointeur (32 ou 64 bits selon la plateforme)

### Méthode step()

```rust
pub fn step(&mut self, dt: f32) {
    // 1. Intégration Verlet
    self.integrate(dt);
    self.integrate_aabbs(dt);

    // 2. Itérations de collision
    for _ in 0..self.collision_iterations {
        self.solve_constraints();
        self.solve_collisions();
        self.solve_box_collisions();
        self.solve_aabb_box_collisions();
    }
}
```

**Le `&mut self`** : en Rust, il faut explicitement dire si on modifie l'objet. `&self` = lecture seule, `&mut self` = modification.

---

## 5. SoA en Rust

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
    pub body_id: Vec<i32>,  // -1 = libre
}
```

Identique au concept Delphi mais avec la syntaxe Rust.

### Accès aux données

```rust
// Delphi : FParticles.PosX[I]
// Rust :
self.particles.pos_x[i]
```

---

## 6. Intégration Verlet

```rust
fn integrate(&mut self, dt: f32) {
    let dt2 = dt * dt;
    let grav_x = self.gravity.x;
    let grav_y = self.gravity.y;
    let damping = self.damping;

    for i in 0..self.particles.len() {
        // Skip les particules fixes
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

**Observation** : le code est quasi identique à Delphi. Rust n'ajoute pas de complexité pour ce genre de code numérique.

---

## 7. FastInvSqrt en Rust

```rust
#[inline]
pub fn fast_inv_sqrt(x: f32) -> f32 {
    let x2 = x * 0.5;

    // unsafe : réinterprétation des bits (type punning)
    let i = unsafe { std::mem::transmute::<f32, u32>(x) };
    let i = 0x5F3759DF - (i >> 1);
    let y = unsafe { std::mem::transmute::<u32, f32>(i) };

    // Newton-Raphson
    y * (1.5 - x2 * y * y)
}
```

**Le mot-clé `unsafe`** : Rust interdit par défaut les opérations qui pourraient causer des bugs mémoire. Ici, le cast de bits `f32 → u32` nécessite `unsafe` car Rust ne peut pas vérifier que c'est intentionnel. C'est, je crois, sûr dans ce contexte.

---

## 8. Collision avec BodyID

```rust
fn collide_particles(&mut self, i: usize, j: usize) {
    // Skip si même corps rigide
    let body_i = self.particles.body_id[i];
    let body_j = self.particles.body_id[j];
    if body_i >= 0 && body_i == body_j {
        return;  // Même corps = pas de collision intra-body
    }

    // ... reste de la collision
}
```

Identique à la logique Delphi.

---

## 9. Particularités Rust

### Le borrow checker

C'est LE truc déstabilisant !!
Exemple de code qui ne compile pas :

```rust
fn solve_box_collisions(&mut self) {
    // ❌ NE COMPILE PAS :
    if let Some(cell) = self.box_grid.get_cell(x, y) {
        for k in 0..cell.count {
            // Ici on emprunte self.box_grid en lecture
            self.collide_particle_static_box(i, cell.box_indices[k]);
            // Et là on veut emprunter self en écriture → CONFLIT
        }
    }
}
```

**Solution** : copier les données localement avant de modifier :

```rust
fn solve_box_collisions(&mut self) {
    if let Some(cell) = self.box_grid.get_cell(pos_x, pos_y) {
        // Copie les indices dans un buffer local
        let count = cell.count;
        let mut box_indices = [0usize; 8];
        for k in 0..count {
            box_indices[k] = cell.box_indices[k];
        }

        // Maintenant on peut modifier self librement
        for k in 0..count {
            self.collide_particle_static_box(i, box_indices[k]);
        }
    }
}
```

C'est un peu verbeux mais ça évite les bugs de mémoire. Mieux que cela, cela pousse à revoir quelquefois l'architecture de haut niveau pour éviter les copies.


### Les closures

```rust
// Delphi : for I := 0 to Count - 1 do
// Rust (fonctionnel) :
(0..count).for_each(|i| {
    // ...
});

// Mais la boucle classique marche aussi :
for i in 0..count {
    // ...
}
```

---

## 10. Compilation et Exécution

```bash
# Compiler la bibliothèque
cargo build --release

# Lancer la démo
cargo run --release --bin gs_phy2d_demo

# Tests unitaires
cargo test
```

L'option `--release` active les optimisations. En debug, Rust est assez lent (bounds checking partout).

---

## 11. La Démo avec Macroquad

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

        // Rendu
        clear_background(WHITE);
        render_world(&world);

        next_frame().await;  // ← async/await pour la boucle de rendu
    }
}
```

**Le `async/await`** : Macroquad utilise l'async pour gérer la boucle de rendu. Ça peut surprendre mais c'est juste une convention de la bibliothèque.

---

## Conseils pour Pascaliens

1. **`let` vs `let mut`** : par défaut les variables sont immutables. Utilise `let mut` pour pouvoir modifier.

2. **Pas de nil/null** : utilise `Option<T>` (équivalent de `Maybe` ou nullable).

3. **Pas d'exceptions** : utilise `Result<T, E>` pour les erreurs.

4. **Le `;` est obligatoire** sauf pour la dernière expression d'un bloc (qui devient la valeur de retour).

5. **`::` vs `.`** : `::` pour les fonctions associées (static), `.` pour les méthodes.

6. **Cargo > IDE** : le workflow Rust passe par le terminal. Cargo fait tout : build, test, docs, dependencies.

---

## Conclusion

Le portage de GS.Phy2D en Rust garde la même structure et les mêmes algorithmes que la version Delphi. Les différences sont principalement syntaxiques et liées au borrow checker.

Pour un développeur Pascal expérimenté, la courbe d'apprentissage Rust est raisonnable. Le plus dur n'est pas la syntaxe mais la mentalité : accepter que le compilateur refuse du code qui "devrait" marcher. Une fois passé ce cap, le code bénéficie de la garantie que si ça compile, il n'y aura pas de crash lié à un accès mémoire invalide, enfin, c'est la promesse ;) on verra !
