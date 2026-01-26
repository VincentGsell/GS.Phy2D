# GS.Phy2D - Comparaison Delphi vs Rust

*Analyse comparative des deux implémentations - Version française*

---

## 1. Vue d'Ensemble

| Aspect | Delphi | Rust |
|--------|--------|------|
| Paradigme | OOP + procédural | Multi-paradigme (fonctionnel + impératif) |
| Gestion mémoire | Manuelle + ARC (interfaces) | Ownership + Borrowing |
| Compilation | Rapide | Lent (code optimisé via LLVM) |
| Écosystème | Mature, niche | En croissance, moderne |
| Portabilité | Windows (+ mobile/Linux) | Toutes plateformes |
| Courbe d'apprentissage | Douce | Raide (borrow checker) |

---

## 2. Syntaxe Comparée

### Déclaration de structure

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

**Observation** : très similaire. Rust utilise `snake_case`, Delphi `PascalCase`.

### Boucle d'itération

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

**Observation** : la syntaxe Rust est plus concise. `0..count` est équivalent à `[0, count[`.

### Création de tableau dynamique

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
// ou
let mut arr = vec![0.0; 100];  // 100 éléments à 0.0
arr[0] = 1.0;
```

### Fonction avec retour

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
    y * (1.5 - x2 * y * y)  // Pas de "return" si dernière expression. Assez "cool" :)
}
```

---

## 3. Gestion Mémoire

### Delphi

```pascal
// Records : gestion mémoire manuelle (stack ou bloc contigu)
var
  P: TParticle;  // Alloué sur la pile

// Classes : Create/Free ou ARC (interfaces)
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
// Structs : ownership automatique
let p = Particle::new(...);  // "Possédé" par la variable p
// p est libéré automatiquement quand il sort du scope

// Références : borrowing explicite
let world = PhyWorld::new();
process(&world);      // Emprunte en lecture
modify(&mut world);   // Emprunte en écriture
```

**Le borrow checker** : Rust vérifie à la compilation qu'on ne peut pas avoir simultanément une référence mutable et d'autres références. Ça prévient les data races et les use-after-free.

---

## 4. Pattern SoA

Les deux implémentations utilisent Structure of Arrays :

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

**Identique conceptuellement**, juste la syntaxe qui diffère.

---

## 5. Spatial Hash - Différences Subtiles

### Delphi

```pascal
// Cells = tableau 2D aplati
FCells: array of array of Cardinal;  // Tableaux dynamiques imbriqués

procedure TSpatialHash.Insert(Tag: Cardinal; X, Y, Radius: Single);
begin
  // Accès direct aux cellules
  FCells[CellIdx][FCellCounts[CellIdx]] := Tag;
  Inc(FCellCounts[CellIdx]);
end;
```

### Rust

```rust
// Cells = Vec de Vec
cells: Vec<Vec<usize>>,

fn insert(&mut self, tag: usize, x: f32, y: f32, radius: f32) {
    // push ajoute à la fin
    self.cells[cell_idx].push(tag);
    self.cell_counts[cell_idx] += 1;
}
```

**Différence** : Rust utilise `.push()` qui gère automatiquement le redimensionnement. En Delphi, il faut pré-allouer ou utiliser `SetLength`.

---

## 6. Gestion des Erreurs

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
// Option pour les valeurs optionnelles
fn get_cell(&self, x: f32, y: f32) -> Option<&BoxGridCell> {
    if in_bounds {
        Some(&self.cells[idx])
    } else {
        None
    }
}

// Utilisation
if let Some(cell) = grid.get_cell(x, y) {
    // cell existe
} else {
    // hors limites
}
```

**Philosophie différente** : Delphi utilise des exceptions (runtime), Rust encode les erreurs dans les types (compile-time).

---

## 7. Performances

### Compilation

| | Delphi | Rust |
|---|---|---|
| Build incrémental | ~1-2s | ~3-10s |
| Build complet | ~5-10s | ~30-60s |
| Code généré | Bon | Excellent |

Rust compile plus lentement mais produit un code plus optimisé grâce à LLVM.

### Exécution

- **Delphi** : bounds checking optionnel, inline agressif
- **Rust** : bounds checking par défaut (désactivable), SIMD auto-vectorisation meilleure

Pour GS.Phy2D, les performances sont meilleurs en Rust (si on se réfère au FPS, on est proche d'un facteur 5/6)
Mais il faut dire que Delphi ne bénéficie pas d'un backend graphique aussi rapide. Je ferai un portage sur un backend 3d à l'occasion.

---

## 8. Ce qui est Plus Simple

### En Delphi

- **Rendu FMX** : intégré, drag & drop d'interface
- **Prototypage rapide** : cycle compile/run très court
- **Interop FMX** : Multiplateforme natif.
- **IDE intégré** : debugger, profiler, designer

### En Rust

- **Gestion des dépendances** : `cargo add macroquad` et c'est fini
- **Documentation** : `cargo doc --open` génère une doc HTML
- **Tests** : `#[test]` intégré, pas besoin de framework externe
- **Cross-compilation** : une commande pour cibler Linux depuis Windows
- **Garanties mémoire** : pas de memory leaks, pas de data races

---

## 9. Ce qui est Plus Compliqué

### En Delphi

- **Gestion mémoire** : Manuel. (pas d'interface ici, j'en mettrais peut être)
- **Multithreading** : Approche un peu passée. J'ai évité les génériques, ciblant les perf. Menace de race conditions toujours présente.
- **Licences** : coût non négligeable

### En Rust

- **Borrow checker** : le compilateur refuse du code valide conceptuellement
- **Lifetime annotations** : `fn foo<'a>(x: &'a str) -> &'a str`
- **Async** : le modèle async de Rust est puissant mais complexe
- **Écosystème gaming** : moins mature que Unity/Unreal/Delphi

---

## 10. Code Identique vs Différent

### Identique (~80%)

- Intégration Verlet
- Tests de collision (cercle-cercle, AABB-AABB)
- Résolution des contraintes
- Calculs d'impulsion
- Structure générale du World

### Différent (~20%)

- Gestion des emprunts (copie locale des indices dans solve_box_collisions)
- Encodage des tags (32 bits en Delphi, 64 bits en Rust)
- Gestion des cellules dans le spatial hash (pré-allocation vs push)
- API publique (getters explicites en Rust)

---

## 11. Verdict

### Delphi si...

- Tu cibles principalement Windows
- Tu as besoin d'une interface graphique native rapidement
- Tu travailles sur un projet existant en Delphi
- Le temps de compilation compte pour ton workflow
- Tu préfères un IDE tout-en-un

### Rust si...

- Tu veux des garanties de sécurité mémoire
- Tu cibles plusieurs plateformes (Linux, WASM, embedded)
- Tu veux un écosystème de packages moderne
- Tu acceptes une courbe d'apprentissage plus raide
- Tu préfères la ligne de commande et des éditeurs modernes (VS Code)

### Pour GS.Phy2D spécifiquement

- Les deux versions fonctionnent de manière équivalente.
- Avec une performance accrue côté Rust, mais c'est une histoire de compilateur.
- Je pense garder ces projets dans les 2 langages pour le futur, même si Rust a un bel avenir, a priori, à l'heure où j'écris ces lignes.

---

## Conclusion

Le portage de GS.Phy2D montre que les algorithmes de physique sont **indépendants du langage**. La vraie différence est dans la **philosophie de gestion mémoire** et l'**écosystème**.

Delphi : "fais ce que je dis". 
Rust : "prouve-moi que c'est sûr". ;)
