# GS.Phy2D - Algorithmes et Mathématiques

*Guide technique - Version française*

---

## 1. Intégration Verlet

### Pourquoi Verlet plutôt qu'Euler ?

L'intégration d'Euler explicite (`pos += vel * dt`) devient instable au-delà d'un certain nombre d'objets ou quand les forces augmentent. En physique de jeu, ça se traduit par des objets qui "explosent" ou traversent les murs.

L'intégration Verlet :

```
new_pos = pos + (pos - old_pos) * damping + accel * dt²
```

On ne stocke pas la vélocité explicitement. Elle est *implicite* dans la différence `pos - old_pos`. Cela a pour effet de :

1. Éviter l'accumulation d'erreurs de vélocité
2. Permettre de déplacer `pos` directement sans casser la vélocité (utile pour les contraintes)
3. Réduire la sensibilité aux grandes forces

### Application du damping

Le damping (amortissement) simule la friction de l'air :

```
vel = (pos - old_pos) * damping  // damping = 0.99 typiquement
```

Un damping de 0.99 = perte de 1% de vélocité par frame. Ça évite les oscillations infinies.

---

## 2. Détection de Collision Broad Phase : Spatial Hash

### Le problème O(n²)

Comparer chaque objet avec tous les autres = O(n²). Avec 1000 objets : 1 million de tests par frame.

### La solution : hachage spatial

On divise l'espace en cellules. Chaque objet est inséré dans la/les cellule(s) qu'il chevauche. Pour trouver les collisions potentielles, on ne regarde que les cellules voisines.

```
┌───┬───┬───┬───┐
│   │ ● │   │   │   ← Seuls les objets dans les mêmes
├───┼───┼───┼───┤     cellules sont comparés
│   │   │ ● │ ● │
├───┼───┼───┼───┤
│ ● │   │   │   │
└───┴───┴───┴───┘
```

Complexité moyenne : O(n) au lieu de O(n²).

### Taille optimale des cellules

Taille recommandée : ~2× le rayon moyen des objets. Trop petit = objets dans trop de cellules. Trop grand = trop d'objets par cellule.

### L'astuce du tag supérieur

Pour éviter de tester la paire (A,B) puis (B,A), on utilise un encodage de tag. Lors d'une query, on ne retourne que les objets avec un tag **supérieur** au tag de l'objet source. Chaque paire n'est testée qu'une fois.

---

## 3. Narrow Phase : Tests de Collision

### Cercle vs Cercle

```
dist² = (x2-x1)² + (y2-y1)²
collision = dist² < (r1 + r2)²
```

On compare les carrés pour éviter la racine carrée.

### Cercle vs AABB

1. Trouver le point le plus proche du cercle sur l'AABB :
   ```
   closest.x = clamp(circle.x, aabb.min_x, aabb.max_x)
   closest.y = clamp(circle.y, aabb.min_y, aabb.max_y)
   ```

2. Tester la distance cercle-point :
   ```
   dist² = (circle.x - closest.x)² + (circle.y - closest.y)²
   collision = dist² < radius²
   ```

### AABB vs AABB

Test de séparation sur les deux axes :

```
collision = !(a.max_x < b.min_x || a.min_x > b.max_x ||
              a.max_y < b.min_y || a.min_y > b.max_y)
```

---

## 4. Résolution des Collisions

### Séparation

Quand deux objets se chevauchent, on calcule l'overlap et la normale de collision, puis on sépare proportionnellement aux masses inverses :

```
overlap = min_dist - distance
ratio1 = inv_mass1 / (inv_mass1 + inv_mass2)
ratio2 = inv_mass2 / (inv_mass1 + inv_mass2)

pos1 -= normal * overlap * ratio1
pos2 += normal * overlap * ratio2
```

Un objet lourd (petite inv_mass) bouge moins qu'un objet léger.

### Impulsion (rebond)

Après séparation, on calcule la vélocité relative le long de la normale :

```
v_rel = dot(v1 - v2, normal)
```

Si `v_rel > 0`, les objets s'éloignent déjà : rien à faire.

Sinon, on applique une impulsion :

```
j = -(1 + restitution) * v_rel / (inv_mass1 + inv_mass2)
```

En Verlet, au lieu de modifier une vélocité explicite, on modifie `old_pos` :

```
old_pos1 = pos1 - (vel1 - impulse * inv_mass1)
old_pos2 = pos2 - (vel2 + impulse * inv_mass2)
```

---

## 5. Contraintes de Distance

### Principe

Deux particules connectées doivent rester à une distance fixe (`rest_length`). Si elles s'écartent ou se rapprochent, on les corrige :

```
delta = p2 - p1
distance = length(delta)
error = distance - rest_length
correction = delta * (error / distance) * 0.5 * stiffness
```

On applique `correction` à p1 et `-correction` à p2 (proportionnellement aux masses).

### Stiffness

- `1.0` : correction complète = rigide
- `0.5` : correction partielle = semi-élastique
- `0.1` : correction faible = ressort mou

### Corps rigides

Un rectangle rigide = 4 particules aux coins + 6 contraintes :
- 4 côtés (maintiennent le périmètre)
- 2 diagonales (empêchent la déformation en parallélogramme)

```
P0 ─────── P1
│  \   /   │
│   \ /    │
│    X     │   ← Diagonales = rigidité
│   / \    │
│  /   \   │
P3 ─────── P2
```

---

## 6. FastInvSqrt - Racine carrée inverse rapide

### Algorithme connu du jeu vidéo

Utilisé dans Quake III (1999) : 

```c
float fast_inv_sqrt(float x) {
    float x2 = x * 0.5f;
    int i = *(int*)&x;              // Bit hack
    i = 0x5F3759DF - (i >> 1);      // Constante 
    float y = *(float*)&i;
    return y * (1.5f - x2 * y * y); // Newton-Raphson
}
```

### Usage

La normalisation de vecteurs nécessite `1/length = 1/sqrt(x² + y²)`. Avec FastInvSqrt, on évite deux opérations coûteuses (sqrt et division).

Note : sur CPU récents avec SSE/AVX, l'instruction `rsqrtss` fait le même travail en hardware.

---

## 7. Encodage des Tags

Pour identifier le type d'objet dans le spatial hash, on encode type + index dans un seul entier :

```
[3 bits: type][61 bits: index]
```

- Bits 63-61 : type (0 = particule, 1 = AABB, etc.)
- Bits 60-0 : index dans le tableau

Fonctions :
```
encode(type, index) = (type << 61) | index
decode(tag) → (type, index)
```

Évite les structures polymorphes ou les enums avec dispatch dynamique.

---

## 8. Structure of Arrays (SoA)

### Array of Structures (AoS) - Classique

```
particles[i].pos_x
particles[i].pos_y
particles[i].radius
```

En mémoire : `[pos_x, pos_y, radius, pos_x, pos_y, radius, ...]`

### Structure of Arrays (SoA)

```
pos_x[i]
pos_y[i]
radius[i]
```

En mémoire : `[pos_x, pos_x, pos_x, ...][pos_y, pos_y, ...][radius, radius, ...]`

### Pourquoi SoA ?

1. **Cache locality** : quand on itère sur `pos_x`, toutes les valeurs sont contiguës
2. **SIMD** : le CPU peut charger 4/8 valeurs d'un coup
3. **Préfetch** : le CPU prédit mieux les accès mémoire

Gain mesuré : 2-3× sur simulations denses.

---

## Références

- [Verlet Integration](https://en.wikipedia.org/wiki/Verlet_integration) - Wikipedia
- [Fast Inverse Square Root](https://en.wikipedia.org/wiki/Fast_inverse_square_root) - Wikipedia
- [Game Physics Engine Development](https://www.amazon.com/Game-Physics-Engine-Development-Commercial-Grade/dp/0123819768) - Ian Millington
- [Advanced Character Physics](https://www.cs.cmu.edu/afs/cs/academic/class/15462-s13/www/lec_slides/Jakobsen.pdf) - Thomas Jakobsen (GDC 2001)
