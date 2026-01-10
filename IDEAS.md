# GSPhy2D - Ideas & Roadmap

## Effets fluides observés

Avec beaucoup de particules qui interagissent, on observe des comportements émergents similaires à la dynamique des fluides :

- **Convection** : les particules "chaudes" (rouges) montent et les "froides" (bleues) descendent
- **Transfert d'énergie** : quand une particule rapide frappe une lente, on voit le "flash" de couleur se propager
- **Équilibre thermique** : avec le temps, tout le système tend vers le bleu (refroidissement)

## Évolutions possibles vers la simulation de fluides

- **SPH (Smoothed Particle Hydrodynamics)** pour simuler de vrais fluides
- **Pression inter-particules** pour éviter la compression excessive
- **Viscosité** pour un comportement plus "liquide"

---

## Optimisations Performance

### État actuel (après optimisations 2026-01-10)
- ~10000 particules à **24 FPS** (était 20 FPS)
- Structure SoA (Structure of Arrays) implémentée
- Spatial hash avec pointeurs directs + pré-allocation
- Intégration Verlet
- FastInvSqrt (Quake III) pour normalisation rapide

### Optimisations testées (2026-01-10)

| Optimisation | Résultat |
|-------------|----------|
| Directives `{$O+}` `{$R-}` `{$Q-}` | ✅ Inclus |
| Pré-allocation buffers | ✅ Inclus |
| FastInvSqrt | ✅ Inclus |
| **SoA (Structure of Arrays)** | ✅ **+20% (20→24 FPS)** |
| Multithreading TParallel.For | ❌ Perte de perfs (overhead) |
| Multithreading chunks manuels | ❌ Perte de perfs (création threads) |
| Sort and Sweep (SAP) | ❌ 6 FPS (trop de paires dans espace confiné) |

**Conclusion** : Le SoA est le seul gain significatif. Le multithreading et SAP ne fonctionnent pas bien pour ce cas d'usage (particules denses dans un espace confiné).

### Prochaines étapes d'optimisation (non testées)

#### 1. SIMD (SSE/AVX) - Gain estimé: 2-4x
Vectoriser les calculs sur 4 ou 8 particules simultanément :
```pascal
// Au lieu de traiter 1 particule à la fois
// Traiter 4 positions X en parallèle avec SSE
// Traiter 8 positions X en parallèle avec AVX
```
- Regrouper les données par composante (SoA au lieu de AoS)
- Utiliser les intrinsics SSE/AVX de Delphi

#### 2. Structure SoA (Structure of Arrays) - Gain estimé: 1.5-2x
Actuellement (AoS - Array of Structures):
```pascal
TPhyParticle = record
  Pos: TVec2;      // X, Y ensemble
  OldPos: TVec2;
  Radius: Single;
end;
FParticles: array of TPhyParticle;
```

Optimisé (SoA - Structure of Arrays):
```pascal
FPosX: array of Single;      // Tous les X ensemble (cache-friendly)
FPosY: array of Single;      // Tous les Y ensemble
FOldPosX: array of Single;
FOldPosY: array of Single;
FRadius: array of Single;
```
Meilleure utilisation du cache CPU car accès séquentiels.

#### 3. Multithreading - Gain estimé: 2-8x (selon nb de coeurs)
- **Phase 1 - Intégration** : parallélisable à 100% (chaque particule indépendante)
- **Phase 2 - Spatial Hash rebuild** : parallélisable avec locks par cellule
- **Phase 3 - Collision detection** : diviser l'espace en zones traitées en parallèle
- **Phase 4 - Collision response** : plus délicat, nécessite synchronisation

Utiliser `TParallel.For` de Delphi ou threads manuels.

#### 4. Éviter les allocations dynamiques
- Pré-allouer `FQueryResult` à une taille fixe max
- Éviter `SetLength` pendant la simulation
- Pool d'objets si nécessaire

#### 5. Optimisation du Spatial Hash
- **Taille de cellule optimale** : 2x le rayon max (déjà fait)
- **Grille plate** au lieu de 2D : `FCells: array of TCell` avec index = Y * Width + X
- **Skip des cellules vides** : maintenir une liste des cellules non-vides

#### 6. Broad Phase améliorée
- **Sort and Sweep** sur l'axe X pour réduire les paires à tester
- **Hierarchical Grid** : grille grossière + grille fine

#### 7. Réduction des calculs
- **Éviter Sqrt** quand possible (comparer distances au carré)
- **Fast inverse sqrt** pour la normalisation
- **Lookup tables** pour certaines fonctions

#### 8. Compilation optimisée
- Activer les optimisations Delphi : `{$O+}` `{$R-}` `{$Q-}`
- Compiler en 64-bit pour plus de registres
- Utiliser `inline` sur les fonctions critiques (déjà fait sur certaines)

### Ordre de priorité recommandé

1. **SoA + SIMD** - Plus gros gain potentiel
2. **Multithreading Intégration** - Facile à implémenter, bon gain
3. **Éviter allocations** - Petit gain mais facile
4. **Multithreading Collisions** - Plus complexe mais gros gain

### Benchmark cible
- 10000 particules @ 60 FPS (actuel: 24 FPS)
- 50000 particules @ 30 FPS
- 100000 particules @ 15 FPS

---

## GPU Compute (piste future)

Pour dépasser les limites du CPU, le GPU est la seule option réaliste pour 100k+ particules.

### Options en Pascal/Delphi

#### 1. Compute Shaders (OpenGL 4.3+)
- Écrire la physique en GLSL
- Utiliser des SSBOs (Shader Storage Buffer Objects) pour les données particules
- Le rendu et la physique restent sur GPU = zéro transfert CPU↔GPU
- **Avantage** : Intégré à OpenGL, pas de dépendance externe
- **Inconvénient** : Syntaxe GLSL, débogage difficile

```glsl
// Exemple compute shader pour intégration Verlet
#version 430
layout(local_size_x = 256) in;

layout(std430, binding = 0) buffer PosBuffer { vec2 positions[]; };
layout(std430, binding = 1) buffer OldPosBuffer { vec2 oldPositions[]; };

uniform vec2 gravity;
uniform float dt;
uniform float damping;

void main() {
    uint i = gl_GlobalInvocationID.x;
    vec2 vel = (positions[i] - oldPositions[i]) * damping;
    vec2 newPos = positions[i] + vel + gravity * dt * dt;
    oldPositions[i] = positions[i];
    positions[i] = newPos;
}
```

#### 2. OpenCL
- Plus générique (fonctionne sur AMD, NVIDIA, Intel)
- Binding Pascal disponible (OpenCL headers)
- **Avantage** : Portable
- **Inconvénient** : Setup plus complexe, moins intégré au rendu

#### 3. CUDA (NVIDIA seulement)
- Meilleure performance sur cartes NVIDIA
- Bindings Pascal existent
- **Inconvénient** : Vendor lock-in

### Architecture recommandée (Compute Shaders)

```
[CPU]                          [GPU]
  |                              |
  | Upload initial data ------→ | SSBO positions[]
  |                              | SSBO oldPositions[]
  |                              | SSBO radii[]
  |                              |
  | glDispatchCompute() ------→ | Compute Shader: Integrate
  |                              | Compute Shader: BuildGrid (spatial hash)
  |                              | Compute Shader: Collisions
  |                              |
  | (pas de readback!)           | Vertex Shader: Draw from SSBO
  |                              | Fragment Shader: Color by velocity
```

**Clé** : Ne jamais transférer les données vers le CPU. Tout reste sur GPU.

### Complexité de la collision sur GPU

Le spatial hash sur GPU est plus complexe :
1. Calculer la cellule de chaque particule (trivial)
2. Compter les particules par cellule (atomic counters)
3. Prefix sum pour calculer les offsets
4. Remplir la grille triée
5. Pour chaque particule, parcourir les cellules voisines

Des bibliothèques existent (ex: `thrust` pour CUDA) mais pas directement en Pascal.
