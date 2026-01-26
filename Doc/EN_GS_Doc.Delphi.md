# GS.Phy2D - Delphi Implementation

*Pascal/Delphi Implementation Guide - English Version*

---

## 1. Unit Architecture

```
Sources/
├── GS.Phy.Vec2.pas      ← Vector mathematics
├── GS.Phy.Types.pas     ← Base types (TParticle, TConstraint, etc.)
├── GS.Phy.AABB.pas      ← Dynamic AABBs
├── GS.Phy.SpatialHash.pas ← Spatial partitioning
├── GS.Phy.World.pas     ← Main engine
├── GS.Phy.Renderer.pas  ← Abstract rendering class
└── GS.Phy.Renderer.FMX.pas ← FMX2d rendering (LARGE MARGIN for optimization!)
```

### Naming Convention

- `GS.Phy.` prefix for all units
- `T` prefix for types (records and classes)
- `P` prefix for pointers to records
- `SoA` suffix for "Structure of Arrays" structures

---

## 2. Fundamental Types

### TParticle - Particle Record

```pascal
type
  TParticle = record
    PosX, PosY: Single;      // Current position
    OldPosX, OldPosY: Single; // Verlet position
    AccelX, AccelY: Single;   // Acceleration
    Radius: Single;
    InvMass: Single;          // 0 = fixed
    Restitution: Single;
    Flags: Byte;
  end;
  PParticle = ^TParticle;
```

**Why records?** In Delphi, records are allocated on the stack or in contiguous blocks. No memory fragmentation, no reference counting. Suited for physics calculations.

### TParticleSoA - SoA Format

```pascal
type
  TParticleSoA = record
    PosX: array of Single;
    PosY: array of Single;
    OldPosX: array of Single;
    // ... etc
    BodyID: array of Integer;  // -1 = free
    Count: Integer;
  end;
```

Data is accessed by index: `FParticles.PosX[I]` rather than `FParticles[I].PosX`.

---

## 3. TPhyWorld - The Engine

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

### The Step() Loop

```pascal
procedure TPhyWorld.Step(DeltaTime: Single);
begin
  // 1. Verlet integration
  Integrate(DeltaTime);
  IntegrateAABBs(DeltaTime);

  // 2. Resolution (multiple iterations)
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

## 4. Verlet Integration in Delphi

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
    // Skip fixed particles
    if (FParticles.Flags[I] and PHY_FLAG_FIXED) <> 0 then
      Continue;

    // Implicit velocity with damping
    VelX := (FParticles.PosX[I] - FParticles.OldPosX[I]) * FDamping;
    VelY := (FParticles.PosY[I] - FParticles.OldPosY[I]) * FDamping;

    // Add gravity
    FParticles.AccelX[I] := FParticles.AccelX[I] + FGravityX;
    FParticles.AccelY[I] := FParticles.AccelY[I] + FGravityY;

    // Verlet: pos + vel + accel * dt²
    NewPosX := FParticles.PosX[I] + VelX + FParticles.AccelX[I] * DT2;
    NewPosY := FParticles.PosY[I] + VelY + FParticles.AccelY[I] * DT2;

    // Update
    FParticles.OldPosX[I] := FParticles.PosX[I];
    FParticles.OldPosY[I] := FParticles.PosY[I];
    FParticles.PosX[I] := NewPosX;
    FParticles.PosY[I] := NewPosY;

    // Reset acceleration
    FParticles.AccelX[I] := 0;
    FParticles.AccelY[I] := 0;
  end;
end;
```

**Delphi tip**: use local variables (`VelX`, `NewPosX`) rather than accessing arrays multiple times. The compiler can place them in registers.

---

## 5. Spatial Hash

### TSpatialHash

```pascal
type
  TSpatialHash = class
  private
    FCells: array of array of Cardinal;  // Encoded tags
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

### Tag Encoding

```pascal
const
  TAG_PARTICLE = 0;
  TAG_AABB = 1;
  TAG_SHIFT = 28;  // 32 bits: 4 bits type, 28 bits index

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

## 6. Rigid Bodies (BodyID)

### Tip: Use "AABB circles" as base components, associating them via constraints. (see my project 2d-Constraints-Demos.git which illustrates this in a basic way)


### BodyID

```pascal
// In TParticleSoA
BodyID: array of Integer;  // -1 = free particle

// In CollideParticlesSoA
BodyI := FParticles.BodyID[I];
BodyJ := FParticles.BodyID[J];
if (BodyI >= 0) and (BodyI = BodyJ) then
  Exit;  // Same body = no collision
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

  // Create particles
  for I := 0 to High(Circles) do
  begin
    Idx := AddParticle(
      CenterX + Circles[I].X,
      CenterY + Circles[I].Y,
      Circles[I].Z,  // Radius
      False, PartMass, Restitution);
    FParticles.BodyID[Idx] := NewBodyID;
  end;

  // Create constraints between touching circles
  for I := 0 to High(Circles) do
    for J := I + 1 to High(Circles) do
    begin
      DX := Circles[J].X - Circles[I].X;
      DY := Circles[J].Y - Circles[I].Y;
      Dist := Sqrt(DX*DX + DY*DY);
      TouchDist := Circles[I].Z + Circles[J].Z;

      if Dist <= TouchDist * 1.1 then  // 10% margin
        AddConstraint(ParticleStart + I, ParticleStart + J, Stiffness);
    end;

  // ... register the RigidBody
end;
```

---

## 7. FMX Rendering

### Abstract Class

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

### FMX Implementation

```pascal
procedure TPhyRendererFMX.DoDrawLine(X1, Y1, X2, Y2: Single;
  Color: TPhyColor; Thickness: Single);
begin
  if FSceneActive and (FCanvas <> nil) then
  begin
    FCanvas.Stroke.Kind := TBrushKind.Solid;  // Important!
    FCanvas.Stroke.Color := TAlphaColor(Color);
    FCanvas.Stroke.Thickness := Thickness;
    FCanvas.DrawLine(PointF(X1, Y1), PointF(X2, Y2), 1.0);
  end;
end;
```

**FMX pitfall**: if `Stroke.Kind` is not `TBrushKind.Solid`, the line does not display. This is a classic issue.

---

## 8. Delphi Optimizations

### Compilation

- Enable optimizations (`{$O+}`)
- Disable range checking in release (`{$R-}`)
- Disable overflow checking (`{$Q-}`)

### Code

- Prefer `Single` over `Double` (cache-friendly)
- Use `inline` for small functions
- Local variables > repeated array accesses
- `Continue` rather than `if not ... then begin ... end`

### TODO
- Change the relatively slow graphics frontend (Switch to a 3D backend)

### FastInvSqrt in Delphi

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

The Delphi implementation of GS.Phy2D favors:
- **Records** to avoid memory management
- **SoA** for cache performance
- **Pointer arithmetic** when necessary
- **FastInvSqrt** for vector calculations
