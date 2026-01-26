# GS.Phy2D - Implémentation Delphi

*Guide d'implémentation Pascal/Delphi - Version française*

---

## 1. Architecture des Unités

```
Sources/
├── GS.Phy.Vec2.pas      ← Mathématiques vectorielles
├── GS.Phy.Types.pas     ← Types de base (TParticle, TConstraint, etc.)
├── GS.Phy.AABB.pas      ← AABB dynamiques
├── GS.Phy.SpatialHash.pas ← Partitionnement spatial
├── GS.Phy.World.pas     ← Moteur principal
├── GS.Phy.Renderer.pas  ← Classe abstraite de rendu
└── GS.Phy.Renderer.FMX.pas ← Rendu FMX2d (GROSSE MARGE pour l'optimisation !)
```

### Convention de nommage

- Préfixe `GS.Phy.` pour toutes les unités
- Préfixe `T` pour les types (records et classes)
- Préfixe `P` pour les pointeurs vers records
- Suffixe `SoA` pour les structures "Structure of Arrays"

---

## 2. Types Fondamentaux

### TParticle - Record de particule

```pascal
type
  TParticle = record
    PosX, PosY: Single;      // Position actuelle
    OldPosX, OldPosY: Single; // Position Verlet
    AccelX, AccelY: Single;   // Accélération
    Radius: Single;
    InvMass: Single;          // 0 = fixé
    Restitution: Single;
    Flags: Byte;
  end;
  PParticle = ^TParticle;
```

**Pourquoi des records ?** En Delphi, les records sont alloués sur la pile ou en bloc contigu. Pas de fragmentation mémoire, pas de référence comptée. Adapté aux calculs physiques.

### TParticleSoA - Format SoA

```pascal
type
  TParticleSoA = record
    PosX: array of Single;
    PosY: array of Single;
    OldPosX: array of Single;
    // ... etc
    BodyID: array of Integer;  // -1 = libre
    Count: Integer;
  end;
```

On accède aux données par index : `FParticles.PosX[I]` plutôt que `FParticles[I].PosX`.

---

## 3. TPhyWorld - Le moteur

### Structure

```pascal
type
  TPhyWorld = class
  private
    FParticles: TParticleSoA;
    FAABBs: TAABBSoA;
    FBoxes: array of TStaticBox;
    FConstraints: array of TConstraint;
    FSpatialHash: TSpatialHash;
    FBoxGrid: TBoxGrid;

    FGravityX, FGravityY: Single;
    FDamping: Single;
    FRestitution: Single;
    FWorldWidth, FWorldHeight: Single;
    FCollisionIterations: Integer;
  public
    procedure Step(DeltaTime: Single);
    function AddParticle(...): Integer;
    function AddConstraint(P1, P2: Integer; Stiffness: Single): Integer;
    // ...
  end;
```

### La boucle Step()

```pascal
procedure TPhyWorld.Step(DeltaTime: Single);
begin
  // 1. Intégration Verlet
  Integrate(DeltaTime);
  IntegrateAABBs(DeltaTime);

  // 2. Résolution (plusieurs itérations)
  for Iter := 0 to FCollisionIterations - 1 do
  begin
    SolveConstraints;
    SolveCollisions;
    SolveBoxCollisions;
    SolveAABBBoxCollisions;
  end;
end;
```

---

## 4. Intégration Verlet en Delphi

```pascal
procedure TPhyWorld.Integrate(DT: Single);
var
  I: Integer;
  VelX, VelY, NewPosX, NewPosY: Single;
  DT2: Single;
begin
  DT2 := DT * DT;

  for I := 0 to FParticles.Count - 1 do
  begin
    // Skip les particules fixes
    if (FParticles.Flags[I] and PHY_FLAG_FIXED) <> 0 then
      Continue;

    // Vélocité implicite avec damping
    VelX := (FParticles.PosX[I] - FParticles.OldPosX[I]) * FDamping;
    VelY := (FParticles.PosY[I] - FParticles.OldPosY[I]) * FDamping;

    // Ajoute gravité
    FParticles.AccelX[I] := FParticles.AccelX[I] + FGravityX;
    FParticles.AccelY[I] := FParticles.AccelY[I] + FGravityY;

    // Verlet : pos + vel + accel * dt²
    NewPosX := FParticles.PosX[I] + VelX + FParticles.AccelX[I] * DT2;
    NewPosY := FParticles.PosY[I] + VelY + FParticles.AccelY[I] * DT2;

    // Update
    FParticles.OldPosX[I] := FParticles.PosX[I];
    FParticles.OldPosY[I] := FParticles.PosY[I];
    FParticles.PosX[I] := NewPosX;
    FParticles.PosY[I] := NewPosY;

    // Reset accélération
    FParticles.AccelX[I] := 0;
    FParticles.AccelY[I] := 0;
  end;
end;
```

**Astuce Delphi** : utiliser des variables locales (`VelX`, `NewPosX`) plutôt que d'accéder plusieurs fois aux tableaux. Le compilateur peut les mettre en registres.

---

## 5. Spatial Hash

### TSpatialHash

```pascal
type
  TSpatialHash = class
  private
    FCells: array of array of Cardinal;  // Tag encodés
    FCellCounts: array of Integer;
    FCellSize: Single;
    FInvCellSize: Single;
    FGridWidth, FGridHeight: Integer;
  public
    procedure Setup(WorldW, WorldH, CellSize: Single);
    procedure Clear;
    procedure Insert(Tag: Cardinal; X, Y, Radius: Single);
    function Query(SelfTag: Cardinal; X, Y, Radius: Single): Integer;
  end;
```

### Encodage des tags

```pascal
const
  TAG_PARTICLE = 0;
  TAG_AABB = 1;
  TAG_SHIFT = 28;  // 32 bits : 4 bits type, 28 bits index

function EncodeTag(ObjType, Index: Integer): Cardinal;
begin
  Result := (ObjType shl TAG_SHIFT) or Cardinal(Index);
end;

procedure DecodeTag(Tag: Cardinal; out ObjType, Index: Integer);
begin
  ObjType := Tag shr TAG_SHIFT;
  Index := Tag and $0FFFFFFF;
end;
```

---

## 6. Corps Rigides (BodyID)

### Astuce : Utiliser les "cercles AABB" comme composant de base, en les associant via des contraintes. (voir mon projet contraintes 2d-Constraints-Demos.git qui illustre cela de manière basique)


### BodyID

```pascal
// Dans TParticleSoA
BodyID: array of Integer;  // -1 = particule libre

// Dans CollideParticlesSoA
BodyI := FParticles.BodyID[I];
BodyJ := FParticles.BodyID[J];
if (BodyI >= 0) and (BodyI = BodyJ) then
  Exit;  // Même corps = pas de collision
```

### AddRigidBody

```pascal
function TPhyWorld.AddRigidBody(CenterX, CenterY: Single;
  const Circles: array of TVec3; Mass, Restitution, Stiffness: Single): Integer;
var
  I, J, ParticleStart, Idx: Integer;
  DX, DY, Dist, TouchDist: Single;
  PartMass: Single;
  NewBodyID: Integer;
begin
  ParticleStart := FParticles.Count;
  PartMass := Mass / Length(Circles);
  NewBodyID := Length(FRigidBodies);

  // Crée les particules
  for I := 0 to High(Circles) do
  begin
    Idx := AddParticle(
      CenterX + Circles[I].X,
      CenterY + Circles[I].Y,
      Circles[I].Z,  // Rayon
      False, PartMass, Restitution);
    FParticles.BodyID[Idx] := NewBodyID;
  end;

  // Crée les contraintes entre cercles qui se touchent
  for I := 0 to High(Circles) do
    for J := I + 1 to High(Circles) do
    begin
      DX := Circles[J].X - Circles[I].X;
      DY := Circles[J].Y - Circles[I].Y;
      Dist := Sqrt(DX*DX + DY*DY);
      TouchDist := Circles[I].Z + Circles[J].Z;

      if Dist <= TouchDist * 1.1 then  // 10% de marge
        AddConstraint(ParticleStart + I, ParticleStart + J, Stiffness);
    end;

  // ... enregistre le RigidBody
end;
```

---

## 7. Rendu FMX

### Classe abstraite

```pascal
type
  TPhyRenderer = class abstract
  public
    procedure BeginScene; virtual; abstract;
    procedure EndScene; virtual; abstract;
    procedure DrawCircle(X, Y, Radius: Single; Color: TPhyColor); virtual; abstract;
    procedure DrawRect(X, Y, W, H: Single; Color: TPhyColor); virtual; abstract;
    procedure DrawLine(X1, Y1, X2, Y2: Single; Color: TPhyColor; Thickness: Single); virtual; abstract;
  end;
```

### Implémentation FMX

```pascal
procedure TPhyRendererFMX.DoDrawLine(X1, Y1, X2, Y2: Single;
  Color: TPhyColor; Thickness: Single);
begin
  if FSceneActive and (FCanvas <> nil) then
  begin
    FCanvas.Stroke.Kind := TBrushKind.Solid;  // Important !
    FCanvas.Stroke.Color := TAlphaColor(Color);
    FCanvas.Stroke.Thickness := Thickness;
    FCanvas.DrawLine(PointF(X1, Y1), PointF(X2, Y2), 1.0);
  end;
end;
```

**Piège FMX** : si `Stroke.Kind` n'est pas `TBrushKind.Solid`, la ligne ne s'affiche pas. C'est un classique.

---

## 8. Optimisations Delphi

### Compilation

- Activer les optimisations (`{$O+}`)
- Désactiver le range checking en release (`{$R-}`)
- Désactiver le overflow checking (`{$Q-}`)

### Code

- Préférer `Single` à `Double` (cache-friendly)
- Utiliser `inline` pour les petites fonctions
- Variables locales > accès tableaux répétés
- `Continue` plutôt que `if not ... then begin ... end`

### TODO
- Changer le front graphiques relativement lent (Passer sur un backend 3d)

### FastInvSqrt en Delphi

```pascal
function FastInvSqrt(X: Single): Single;
var
  I: Cardinal;
  X2, Y: Single;
begin
  X2 := X * 0.5;
  I := PCardinal(@X)^;
  I := $5F3759DF - (I shr 1);
  Y := PSingle(@I)^;
  Result := Y * (1.5 - X2 * Y * Y);
end;
```

---

## Conclusion

L'implémentation Delphi de GS.Phy2D privilégie :
- **Records** pour éviter la gestion mémoire
- **SoA** pour les performances cache
- **Pointeurs arithmétiques** quand nécessaire
- **FastInvSqrt** pour les calculs vectoriels

