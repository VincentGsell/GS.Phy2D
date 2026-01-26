{
GS.Phy2D - open source 2D physics engine
2026, Vincent Gsell

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

Created by Vincent Gsell [https://github.com/VincentGsell]
}

//History
//20260110 - Created.
//20260110 - SoA optimization for cache-friendly access.


unit GS.Phy.World;
{$IFDEF FPC}
{$MODE DELPHI}
{$ENDIF}

// Compiler optimizations
{$O+}  // Optimizations enabled
{$R-}  // Range checking disabled
{$Q-}  // Overflow checking disabled

interface

uses
  GS.Phy.Vec2,
  GS.Phy.Types,
  GS.Phy.AABB,
  GS.Phy.SpatialHash;

const
  BOX_GRID_MAX_PER_CELL = 8;

  // Object type tags for spatial hash (2 bits = 4 types max)
  TAG_PARTICLE = 0;
  TAG_AABB     = 1;
  TAG_MESH     = 3;  // Reserved for future
  TAG_SHIFT    = 29;
  TAG_MASK     = $E0000000;  // Top 3 bits
  INDEX_MASK   = $1FFFFFFF;  // Bottom 29 bits

type
  // Spatial hash for static boxes - fixed array of pointers per cell
  TBoxGridCell = record
    Boxes: array[0..BOX_GRID_MAX_PER_CELL-1] of PPhyBox;
    Count: Integer;
  end;

  // SoA structure for particles - cache-friendly access
  TParticleSoA = record
    PosX: array of Single;
    PosY: array of Single;
    OldPosX: array of Single;
    OldPosY: array of Single;
    AccelX: array of Single;
    AccelY: array of Single;
    Radius: array of Single;
    InvMass: array of Single;
    Restitution: array of Single;
    Flags: array of Byte;
    BodyID: array of Integer;  // -1 = free particle, >= 0 = belongs to rigid body
  end;

  TPhyWorld = class
  private
    // SoA for particles
    FParticles: TParticleSoA;
    FParticleCount: Integer;
    FParticleCapacity: Integer;

    // SoA for dynamic AABBs
    FAABBs: TAABBSoA;
    FAABBCount: Integer;
    FAABBCapacity: Integer;

    // Temp cache for GetParticle/GetAABB compatibility
    FTempParticle: TPhyParticle;
    FTempAABB: TPhyAABB;

    FBoxes: array of TPhyBox;
    FBoxCount: Integer;

    FConstraints: array of TPhyConstraint;
    FConstraintCount: Integer;

    FRigidBodies: array of TPhyRigidBody;
    FRigidBodyCount: Integer;

    FSpatialHash: TPhySpatialHash;
    FGravity: TVec2;
    FDamping: Single;
    FWorldWidth, FWorldHeight: Single;

    FCollisionIterations: Integer;
    FConstraintIterations: Integer;
    FRestitution: Single;

    // Spatial hash for static boxes
    FBoxGrid: array of TBoxGridCell;
    FBoxGridWidth, FBoxGridHeight: Integer;
    FBoxGridCellSize: Single;
    FBoxGridInvCellSize: Single;
    FBoxGridDirty: Boolean;

    procedure GrowParticleArrays;
    procedure GrowAABBArrays;
    procedure Integrate(DT: Single);
    procedure IntegrateAABBs(DT: Single);
    procedure SolveCollisions;
    procedure SolveConstraints;
    procedure SolveBoxCollisions;
    procedure SolveAABBBoxCollisions;
    procedure CollideParticlesSoA(I, J: Integer);
    procedure CollideAABBsSoA(I, J: Integer);
    procedure CollideParticleAABBSoA(PI, AI: Integer);
    procedure RebuildBoxGrid;
    procedure CollideParticleBoxSoA(I: Integer; Box: PPhyBox);
    procedure CollideAABBBoxSoA(I: Integer; Box: PPhyBox);
  public
    constructor Create;
    destructor Destroy; override;

    procedure SetWorldBounds(Width, Height: Single);
    procedure SetGravity(X, Y: Single);

    function AddParticle(X, Y, Radius: Single; Fixed: Boolean = False;
      Mass: Single = 1.0; Restitution: Single = 0.5): Integer;
    function AddAABB(X, Y, Width, Height: Single; Fixed: Boolean = False;
      Mass: Single = 1.0; Restitution: Single = 0.5): Integer;
    function AddBox(X, Y, Width, Height: Single; Restitution: Single = 0.3): Integer;
    function AddConstraint(P1, P2: Integer; Stiffness: Single = 1.0): Integer;

    // Create a rigid rectangle using particles + constraints
    // Subdivisions: 0 = 4 particles (corners), 1 = 8 particles, 2 = 12 particles, etc.
    // Returns index of first particle
    function AddRigidRect(X, Y, Width, Height: Single; ParticleRadius: Single = 3.0;
      Mass: Single = 1.0; Restitution: Single = 0.5; Stiffness: Single = 1.0;
      Subdivisions: Integer = 0): Integer;

    // Create a rigid body from circles (compound collider)
    // Circles: array of TCircleDef (relative X, Y, Radius)
    // Returns rigid body index
    function AddRigidBody(CenterX, CenterY: Single;
      const Circles: array of TVec3;  // X, Y, Radius per circle (relative to center)
      Mass: Single = 1.0; Restitution: Single = 0.5; Stiffness: Single = 1.0): Integer;

    // Helper: create rectangle body from 2 main circles + filler circles
    function AddRectBody(X, Y, Width, Height: Single;
      Mass: Single = 1.0; Restitution: Single = 0.5): Integer;

    function GetRigidBody(Index: Integer): PPhyRigidBody; inline;
    property RigidBodyCount: Integer read FRigidBodyCount;

    procedure Step(DT: Single);
    procedure Clear;

    function GetParticle(Index: Integer): PPhyParticle;
    function GetAABB(Index: Integer): PPhyAABB;
    function GetBox(Index: Integer): PPhyBox; inline;

    // Direct SoA access for particles (faster for rendering)
    function GetPosX(Index: Integer): Single; inline;
    function GetPosY(Index: Integer): Single; inline;
    function GetRadius(Index: Integer): Single; inline;
    function GetOldPosX(Index: Integer): Single; inline;
    function GetOldPosY(Index: Integer): Single; inline;

    // Direct SoA access for AABBs
    function GetAABBPosX(Index: Integer): Single; inline;
    function GetAABBPosY(Index: Integer): Single; inline;
    function GetAABBOldPosX(Index: Integer): Single; inline;
    function GetAABBOldPosY(Index: Integer): Single; inline;
    function GetAABBHalfW(Index: Integer): Single; inline;
    function GetAABBHalfH(Index: Integer): Single; inline;

    // Constraint access for rendering
    function GetConstraint(Index: Integer): PPhyConstraint; inline;

    property ParticleCount: Integer read FParticleCount;
    property AABBCount: Integer read FAABBCount;
    property BoxCount: Integer read FBoxCount;
    property ConstraintCount: Integer read FConstraintCount;
    property CollisionIterations: Integer read FCollisionIterations write FCollisionIterations;
    property ConstraintIterations: Integer read FConstraintIterations write FConstraintIterations;
    property Damping: Single read FDamping write FDamping;
    property Restitution: Single read FRestitution write FRestitution;
  end;

implementation

const
  INITIAL_CAPACITY = 256;

procedure TPhyWorld.GrowParticleArrays;
begin
  FParticleCapacity := FParticleCapacity * 2;
  SetLength(FParticles.PosX, FParticleCapacity);
  SetLength(FParticles.PosY, FParticleCapacity);
  SetLength(FParticles.OldPosX, FParticleCapacity);
  SetLength(FParticles.OldPosY, FParticleCapacity);
  SetLength(FParticles.AccelX, FParticleCapacity);
  SetLength(FParticles.AccelY, FParticleCapacity);
  SetLength(FParticles.Radius, FParticleCapacity);
  SetLength(FParticles.InvMass, FParticleCapacity);
  SetLength(FParticles.Restitution, FParticleCapacity);
  SetLength(FParticles.Flags, FParticleCapacity);
  SetLength(FParticles.BodyID, FParticleCapacity);
end;

procedure TPhyWorld.GrowAABBArrays;
begin
  FAABBCapacity := FAABBCapacity * 2;
  SetLength(FAABBs.PosX, FAABBCapacity);
  SetLength(FAABBs.PosY, FAABBCapacity);
  SetLength(FAABBs.OldPosX, FAABBCapacity);
  SetLength(FAABBs.OldPosY, FAABBCapacity);
  SetLength(FAABBs.AccelX, FAABBCapacity);
  SetLength(FAABBs.AccelY, FAABBCapacity);
  SetLength(FAABBs.HalfW, FAABBCapacity);
  SetLength(FAABBs.HalfH, FAABBCapacity);
  SetLength(FAABBs.InvMass, FAABBCapacity);
  SetLength(FAABBs.Restitution, FAABBCapacity);
  SetLength(FAABBs.Flags, FAABBCapacity);
end;

constructor TPhyWorld.Create;
begin
  inherited;
  FParticleCount := 0;
  FParticleCapacity := INITIAL_CAPACITY;

  // Allocate particle SoA arrays
  SetLength(FParticles.PosX, FParticleCapacity);
  SetLength(FParticles.PosY, FParticleCapacity);
  SetLength(FParticles.OldPosX, FParticleCapacity);
  SetLength(FParticles.OldPosY, FParticleCapacity);
  SetLength(FParticles.AccelX, FParticleCapacity);
  SetLength(FParticles.AccelY, FParticleCapacity);
  SetLength(FParticles.Radius, FParticleCapacity);
  SetLength(FParticles.InvMass, FParticleCapacity);
  SetLength(FParticles.Restitution, FParticleCapacity);
  SetLength(FParticles.Flags, FParticleCapacity);
  SetLength(FParticles.BodyID, FParticleCapacity);

  // Allocate AABB SoA arrays
  FAABBCount := 0;
  FAABBCapacity := INITIAL_CAPACITY;
  SetLength(FAABBs.PosX, FAABBCapacity);
  SetLength(FAABBs.PosY, FAABBCapacity);
  SetLength(FAABBs.OldPosX, FAABBCapacity);
  SetLength(FAABBs.OldPosY, FAABBCapacity);
  SetLength(FAABBs.AccelX, FAABBCapacity);
  SetLength(FAABBs.AccelY, FAABBCapacity);
  SetLength(FAABBs.HalfW, FAABBCapacity);
  SetLength(FAABBs.HalfH, FAABBCapacity);
  SetLength(FAABBs.InvMass, FAABBCapacity);
  SetLength(FAABBs.Restitution, FAABBCapacity);
  SetLength(FAABBs.Flags, FAABBCapacity);

  FBoxCount := 0;
  SetLength(FBoxes, 16);

  FConstraintCount := 0;
  SetLength(FConstraints, 64);

  FRigidBodyCount := 0;
  SetLength(FRigidBodies, 16);

  FSpatialHash := TPhySpatialHash.Create;
  FGravity := Vec2(0, 500);
  FDamping := 0.99;
  FWorldWidth := 800;
  FWorldHeight := 600;

  FCollisionIterations := 2;
  FConstraintIterations := 1;
  FRestitution := 0.3;

  FSpatialHash.Setup(FWorldWidth, FWorldHeight, 32);

  FBoxGridCellSize := 32;
  FBoxGridInvCellSize := 1 / FBoxGridCellSize;
  FBoxGridWidth := 0;
  FBoxGridHeight := 0;
  FBoxGridDirty := True;
end;

destructor TPhyWorld.Destroy;
begin
  FSpatialHash.Free;
  inherited;
end;

procedure TPhyWorld.SetWorldBounds(Width, Height: Single);
var
  MaxRadius: Single;
  I: Integer;
begin
  FWorldWidth := Width;
  FWorldHeight := Height;

  MaxRadius := 10;
  for I := 0 to FParticleCount - 1 do
    if FParticles.Radius[I] > MaxRadius then
      MaxRadius := FParticles.Radius[I];

  FSpatialHash.Setup(Width, Height, MaxRadius * 2);
  FBoxGridDirty := True;
end;

procedure TPhyWorld.SetGravity(X, Y: Single);
begin
  FGravity := Vec2(X, Y);
end;

function TPhyWorld.AddParticle(X, Y, Radius: Single; Fixed: Boolean;
  Mass: Single; Restitution: Single): Integer;
var
  Flags: Byte;
begin
  if FParticleCount >= FParticleCapacity then
    GrowParticleArrays;

  FParticles.PosX[FParticleCount] := X;
  FParticles.PosY[FParticleCount] := Y;
  FParticles.OldPosX[FParticleCount] := X;
  FParticles.OldPosY[FParticleCount] := Y;
  FParticles.AccelX[FParticleCount] := 0;
  FParticles.AccelY[FParticleCount] := 0;
  FParticles.Radius[FParticleCount] := Radius;

  if Fixed or (Mass <= 0) then
    FParticles.InvMass[FParticleCount] := 0
  else
    FParticles.InvMass[FParticleCount] := 1.0 / Mass;

  FParticles.Restitution[FParticleCount] := Restitution;

  Flags := PHY_FLAG_COLLIDABLE;
  if Fixed then
    Flags := Flags or PHY_FLAG_FIXED;
  FParticles.Flags[FParticleCount] := Flags;
  FParticles.BodyID[FParticleCount] := -1;  // Free particle by default

  Result := FParticleCount;
  Inc(FParticleCount);
end;

function TPhyWorld.AddAABB(X, Y, Width, Height: Single; Fixed: Boolean;
  Mass: Single; Restitution: Single): Integer;
var
  Flags: Byte;
begin
  if FAABBCount >= FAABBCapacity then
    GrowAABBArrays;

  FAABBs.PosX[FAABBCount] := X;
  FAABBs.PosY[FAABBCount] := Y;
  FAABBs.OldPosX[FAABBCount] := X;
  FAABBs.OldPosY[FAABBCount] := Y;
  FAABBs.AccelX[FAABBCount] := 0;
  FAABBs.AccelY[FAABBCount] := 0;
  FAABBs.HalfW[FAABBCount] := Width * 0.5;
  FAABBs.HalfH[FAABBCount] := Height * 0.5;

  if Fixed or (Mass <= 0) then
    FAABBs.InvMass[FAABBCount] := 0
  else
    FAABBs.InvMass[FAABBCount] := 1.0 / Mass;

  FAABBs.Restitution[FAABBCount] := Restitution;

  Flags := PHY_FLAG_COLLIDABLE;
  if Fixed then
    Flags := Flags or PHY_FLAG_FIXED;
  FAABBs.Flags[FAABBCount] := Flags;

  Result := FAABBCount;
  Inc(FAABBCount);
end;

function TPhyWorld.AddBox(X, Y, Width, Height: Single; Restitution: Single): Integer;
begin
  if FBoxCount >= Length(FBoxes) then
    SetLength(FBoxes, Length(FBoxes) * 2);

  FBoxes[FBoxCount] := CreateBox(X, Y, Width, Height, Restitution);
  Result := FBoxCount;
  Inc(FBoxCount);
  FBoxGridDirty := True;
end;

function TPhyWorld.AddConstraint(P1, P2: Integer; Stiffness: Single): Integer;
var
  DX, DY, Dist: Single;
begin
  if FConstraintCount >= Length(FConstraints) then
    SetLength(FConstraints, Length(FConstraints) * 2);

  DX := FParticles.PosX[P2] - FParticles.PosX[P1];
  DY := FParticles.PosY[P2] - FParticles.PosY[P1];
  Dist := Sqrt(DX * DX + DY * DY);

  FConstraints[FConstraintCount].P1 := P1;
  FConstraints[FConstraintCount].P2 := P2;
  FConstraints[FConstraintCount].RestLength := Dist;
  FConstraints[FConstraintCount].Stiffness := Stiffness;

  Result := FConstraintCount;
  Inc(FConstraintCount);
end;

function TPhyWorld.AddRigidRect(X, Y, Width, Height: Single; ParticleRadius: Single = 3.0;
      Mass: Single = 1.0; Restitution: Single = 0.5; Stiffness: Single = 1.0;
      Subdivisions: Integer = 0): Integer;
var
  Particles: array of Integer;
  ParticleCount, I, J, Next: Integer;
  HW, HH: Single;
  PartMass: Single;
  PX, PY: Single;
  PerSide: Integer;  // particles per side (including start corner, excluding end corner)
  T: Single;
begin
  HW := Width / 2;
  HH := Height / 2;

  // PerSide = 1 + Subdivisions (corner + intermediate points)
  PerSide := 1 + Subdivisions;
  ParticleCount := 4 * PerSide;  // 4 sides

  PartMass := Mass / ParticleCount;
  SetLength(Particles, ParticleCount);

  // Create particles around perimeter (clockwise from top-left)
  // Side 0: Top (P0 to P1)
  for I := 0 to PerSide - 1 do
  begin
    T := I / PerSide;
    PX := X - HW + T * Width;
    PY := Y - HH;
    Particles[I] := AddParticle(PX, PY, ParticleRadius, False, PartMass, Restitution);
  end;

  // Side 1: Right (P1 to P2)
  for I := 0 to PerSide - 1 do
  begin
    T := I / PerSide;
    PX := X + HW;
    PY := Y - HH + T * Height;
    Particles[PerSide + I] := AddParticle(PX, PY, ParticleRadius, False, PartMass, Restitution);
  end;

  // Side 2: Bottom (P2 to P3)
  for I := 0 to PerSide - 1 do
  begin
    T := I / PerSide;
    PX := X + HW - T * Width;
    PY := Y + HH;
    Particles[2 * PerSide + I] := AddParticle(PX, PY, ParticleRadius, False, PartMass, Restitution);
  end;

  // Side 3: Left (P3 to P0)
  for I := 0 to PerSide - 1 do
  begin
    T := I / PerSide;
    PX := X - HW;
    PY := Y + HH - T * Height;
    Particles[3 * PerSide + I] := AddParticle(PX, PY, ParticleRadius, False, PartMass, Restitution);
  end;

  // Perimeter constraints (connect each particle to next)
  for I := 0 to ParticleCount - 1 do
  begin
    Next := (I + 1) mod ParticleCount;
    AddConstraint(Particles[I], Particles[Next], Stiffness);
  end;

  // Cross constraints for rigidity (connect each particle to opposite)
  for I := 0 to ParticleCount div 2 - 1 do
  begin
    J := I + ParticleCount div 2;
    AddConstraint(Particles[I], Particles[J], Stiffness);
  end;

  // Additional diagonal constraints (corners)
  // Connect corner 0 to corner 2, corner 1 to corner 3
  AddConstraint(Particles[0], Particles[2 * PerSide], Stiffness);
  AddConstraint(Particles[PerSide], Particles[3 * PerSide], Stiffness);

  Result := Particles[0];
end;

function TPhyWorld.AddRigidBody(CenterX, CenterY: Single;
  const Circles: array of TVec3;
  Mass: Single; Restitution: Single; Stiffness: Single): Integer;
var
  I, J: Integer;
  CircleCount: Integer;
  ParticleStart, ConstraintStart: Integer;
  PartMass: Single;
  Dist, TouchDist: Single;
  DX, DY: Single;
begin
  CircleCount := Length(Circles);
  if CircleCount = 0 then
  begin
    Result := -1;
    Exit;
  end;

  // Grow rigid body array if needed
  if FRigidBodyCount >= Length(FRigidBodies) then
    SetLength(FRigidBodies, Length(FRigidBodies) * 2 + 8);

  ParticleStart := FParticleCount;
  ConstraintStart := FConstraintCount;
  PartMass := Mass / CircleCount;

  // Create particles for each circle
  for I := 0 to CircleCount - 1 do
  begin
    AddParticle(
      CenterX + Circles[I].X,
      CenterY + Circles[I].Y,
      Circles[I].Z,  // Radius
      False,
      PartMass,
      Restitution
    );
    // Assign body ID to this particle
    FParticles.BodyID[ParticleStart + I] := FRigidBodyCount;
  end;

  // Create constraints between circles that touch or overlap
  for I := 0 to CircleCount - 2 do
  begin
    for J := I + 1 to CircleCount - 1 do
    begin
      DX := Circles[J].X - Circles[I].X;
      DY := Circles[J].Y - Circles[I].Y;
      Dist := Sqrt(DX * DX + DY * DY);
      TouchDist := Circles[I].Z + Circles[J].Z;

      // Connect if circles touch or overlap (with small margin)
      if Dist <= TouchDist * 1.1 then
      begin
        AddConstraint(ParticleStart + I, ParticleStart + J, Stiffness);
      end;
    end;
  end;

  // Register rigid body
  FRigidBodies[FRigidBodyCount].ParticleStart := ParticleStart;
  FRigidBodies[FRigidBodyCount].ParticleCount := CircleCount;
  FRigidBodies[FRigidBodyCount].ConstraintStart := ConstraintStart;
  FRigidBodies[FRigidBodyCount].ConstraintCount := FConstraintCount - ConstraintStart;

  Result := FRigidBodyCount;
  Inc(FRigidBodyCount);
end;

function TPhyWorld.AddRectBody(X, Y, Width, Height: Single;
  Mass: Single; Restitution: Single): Integer;
var
  Circles: array of TVec3;
  HW, HH: Single;
  MainRadius, FillerRadius: Single;
begin
  // Main circles at left and right (half the height as radius)
  HW := Width / 2;
  HH := Height / 2;
  MainRadius := HH;  // Main circles cover the height

  // If width > height, we need filler circles between main circles
  if Width > Height then
  begin
    FillerRadius := HH * 0.5;  // Smaller filler circles

    // Count circles: 2 main + fillers in between and on edges
    SetLength(Circles, 6);

    // Left main circle
    Circles[0] := Vec3(-HW + MainRadius, 0, MainRadius);
    // Right main circle
    Circles[1] := Vec3(HW - MainRadius, 0, MainRadius);

    // Filler circles in interstices (top and bottom middle)
    Circles[2] := Vec3(0, -HH + FillerRadius, FillerRadius);  // Top center
    Circles[3] := Vec3(0, HH - FillerRadius, FillerRadius);   // Bottom center

    // Extra fillers for corners
    Circles[4] := Vec3(-HW + FillerRadius, -HH + FillerRadius, FillerRadius);  // Top-left
    Circles[5] := Vec3(HW - FillerRadius, -HH + FillerRadius, FillerRadius);   // Top-right
  end
  else
  begin
    // Simple case: just 2 circles stacked
    MainRadius := HW;
    SetLength(Circles, 2);
    Circles[0] := Vec3(0, -HH + MainRadius, MainRadius);
    Circles[1] := Vec3(0, HH - MainRadius, MainRadius);
  end;

  Result := AddRigidBody(X, Y, Circles, Mass, Restitution, 1.0);
end;

function TPhyWorld.GetRigidBody(Index: Integer): PPhyRigidBody;
begin
  Result := @FRigidBodies[Index];
end;

procedure TPhyWorld.Step(DT: Single);
var
  Iter: Integer;
begin
  Integrate(DT);
  IntegrateAABBs(DT);

  for Iter := 0 to FCollisionIterations - 1 do
  begin
    SolveConstraints;
    SolveCollisions;  // Unified: all dynamic objects via spatial hash
    SolveBoxCollisions;
    SolveAABBBoxCollisions;
  end;
end;

procedure TPhyWorld.Integrate(DT: Single);
var
  I: Integer;
  VelX, VelY, NewPosX, NewPosY: Single;
  DT2, GravX, GravY: Single;
begin
  DT2 := DT * DT;
  GravX := FGravity.X;
  GravY := FGravity.Y;

  // Optimized SoA loop - sequential memory access
  for I := 0 to FParticleCount - 1 do
  begin
    // Skip fixed particles
    if (FParticles.Flags[I] and PHY_FLAG_FIXED) <> 0 then
      Continue;

    // Verlet: new_pos = pos + (pos - old_pos) * damping + accel * dt^2
    VelX := (FParticles.PosX[I] - FParticles.OldPosX[I]) * FDamping;
    VelY := (FParticles.PosY[I] - FParticles.OldPosY[I]) * FDamping;

    // Add gravity to acceleration
    FParticles.AccelX[I] := FParticles.AccelX[I] + GravX;
    FParticles.AccelY[I] := FParticles.AccelY[I] + GravY;

    NewPosX := FParticles.PosX[I] + VelX + FParticles.AccelX[I] * DT2;
    NewPosY := FParticles.PosY[I] + VelY + FParticles.AccelY[I] * DT2;

    FParticles.OldPosX[I] := FParticles.PosX[I];
    FParticles.OldPosY[I] := FParticles.PosY[I];
    FParticles.PosX[I] := NewPosX;
    FParticles.PosY[I] := NewPosY;

    // Reset acceleration
    FParticles.AccelX[I] := 0;
    FParticles.AccelY[I] := 0;
  end;
end;

procedure TPhyWorld.IntegrateAABBs(DT: Single);
var
  I: Integer;
  VelX, VelY, NewPosX, NewPosY: Single;
  DT2, GravX, GravY: Single;
begin
  DT2 := DT * DT;
  GravX := FGravity.X;
  GravY := FGravity.Y;

  for I := 0 to FAABBCount - 1 do
  begin
    if (FAABBs.Flags[I] and PHY_FLAG_FIXED) <> 0 then
      Continue;

    // Verlet integration
    VelX := (FAABBs.PosX[I] - FAABBs.OldPosX[I]) * FDamping;
    VelY := (FAABBs.PosY[I] - FAABBs.OldPosY[I]) * FDamping;

    FAABBs.AccelX[I] := FAABBs.AccelX[I] + GravX;
    FAABBs.AccelY[I] := FAABBs.AccelY[I] + GravY;

    NewPosX := FAABBs.PosX[I] + VelX + FAABBs.AccelX[I] * DT2;
    NewPosY := FAABBs.PosY[I] + VelY + FAABBs.AccelY[I] * DT2;

    FAABBs.OldPosX[I] := FAABBs.PosX[I];
    FAABBs.OldPosY[I] := FAABBs.PosY[I];
    FAABBs.PosX[I] := NewPosX;
    FAABBs.PosY[I] := NewPosY;

    FAABBs.AccelX[I] := 0;
    FAABBs.AccelY[I] := 0;
  end;
end;

function EncodeTag(ObjType, Index: Integer): NativeUInt; inline;
begin
  Result := (NativeUInt(ObjType) shl TAG_SHIFT) or NativeUInt(Index);
end;

procedure DecodeTag(Tag: NativeUInt; out ObjType, Index: Integer); inline;
begin
  ObjType := Integer((Tag and TAG_MASK) shr TAG_SHIFT);
  Index := Integer(Tag and INDEX_MASK);
end;

procedure TPhyWorld.SolveCollisions;
var
  I, K, QueryCount: Integer;
  PosX, PosY, Rad: Single;
  HW, HH: Single;
  Tag: NativeUInt;
  ObjType, ObjIndex: Integer;
  SelfTag: NativeUInt;
begin
  // Rebuild spatial hash with ALL dynamic objects
  FSpatialHash.Clear;

  // Insert particles
  for I := 0 to FParticleCount - 1 do
  begin
    if (FParticles.Flags[I] and PHY_FLAG_COLLIDABLE) <> 0 then
      FSpatialHash.Insert(Pointer(EncodeTag(TAG_PARTICLE, I)),
        FParticles.PosX[I], FParticles.PosY[I], FParticles.Radius[I]);
  end;

  // Insert AABBs with bounding radius
  for I := 0 to FAABBCount - 1 do
  begin
    if (FAABBs.Flags[I] and PHY_FLAG_COLLIDABLE) <> 0 then
    begin
      HW := FAABBs.HalfW[I];
      HH := FAABBs.HalfH[I];
      if HW > HH then Rad := HW else Rad := HH;
      FSpatialHash.Insert(Pointer(EncodeTag(TAG_AABB, I)),
        FAABBs.PosX[I], FAABBs.PosY[I], Rad);
    end;
  end;

  // Query from particles
  for I := 0 to FParticleCount - 1 do
  begin
    if (FParticles.Flags[I] and PHY_FLAG_COLLIDABLE) = 0 then
      Continue;

    PosX := FParticles.PosX[I];
    PosY := FParticles.PosY[I];
    Rad := FParticles.Radius[I];
    SelfTag := EncodeTag(TAG_PARTICLE, I);

    QueryCount := FSpatialHash.Query(Pointer(SelfTag), PosX, PosY, Rad);

    for K := 0 to QueryCount - 1 do
    begin
      Tag := NativeUInt(FSpatialHash.GetQueryResult(K));
      DecodeTag(Tag, ObjType, ObjIndex);

      case ObjType of
        TAG_PARTICLE: CollideParticlesSoA(I, ObjIndex);
        TAG_AABB: CollideParticleAABBSoA(I, ObjIndex);
      end;
    end;
  end;

  // Query from AABBs
  for I := 0 to FAABBCount - 1 do
  begin
    if (FAABBs.Flags[I] and PHY_FLAG_COLLIDABLE) = 0 then
      Continue;

    PosX := FAABBs.PosX[I];
    PosY := FAABBs.PosY[I];
    HW := FAABBs.HalfW[I];
    HH := FAABBs.HalfH[I];
    if HW > HH then Rad := HW else Rad := HH;
    SelfTag := EncodeTag(TAG_AABB, I);

    QueryCount := FSpatialHash.Query(Pointer(SelfTag), PosX, PosY, Rad);

    for K := 0 to QueryCount - 1 do
    begin
      Tag := NativeUInt(FSpatialHash.GetQueryResult(K));
      DecodeTag(Tag, ObjType, ObjIndex);

      case ObjType of
        TAG_AABB: CollideAABBsSoA(I, ObjIndex);
      end;
    end;
  end;
end;

procedure TPhyWorld.CollideParticlesSoA(I, J: Integer);
var
  DeltaX, DeltaY: Single;
  DistSq, Dist, MinDist, Overlap, InvDist: Single;
  NormalX, NormalY: Single;
  TotalInvMass, Ratio1, Ratio2: Single;
  CorrX, CorrY: Single;
  V1X, V1Y, V2X, V2Y: Single;
  RelVel, ImpulseScalar: Single;
  ImpX, ImpY: Single;
  InvMass1, InvMass2: Single;
  BodyI, BodyJ: Integer;
begin
  // Skip collision if both particles belong to the same rigid body
  BodyI := FParticles.BodyID[I];
  BodyJ := FParticles.BodyID[J];
  if (BodyI >= 0) and (BodyI = BodyJ) then
    Exit;

  DeltaX := FParticles.PosX[J] - FParticles.PosX[I];
  DeltaY := FParticles.PosY[J] - FParticles.PosY[I];
  DistSq := DeltaX * DeltaX + DeltaY * DeltaY;
  MinDist := FParticles.Radius[I] + FParticles.Radius[J];

  if DistSq >= MinDist * MinDist then
    Exit;

  if DistSq < 0.0001 then
  begin
    DeltaX := 0.1;
    DeltaY := 0.1;
    DistSq := 0.02;
  end;

  InvDist := FastInvSqrt(DistSq);
  Dist := DistSq * InvDist;
  Overlap := MinDist - Dist;
  NormalX := DeltaX * InvDist;
  NormalY := DeltaY * InvDist;

  InvMass1 := FParticles.InvMass[I];
  InvMass2 := FParticles.InvMass[J];
  TotalInvMass := InvMass1 + InvMass2;
  if TotalInvMass < 0.0001 then
    Exit;

  Ratio1 := InvMass1 / TotalInvMass;
  Ratio2 := InvMass2 / TotalInvMass;

  CorrX := NormalX * Overlap;
  CorrY := NormalY * Overlap;

  // Separate particles
  FParticles.PosX[I] := FParticles.PosX[I] - CorrX * Ratio1;
  FParticles.PosY[I] := FParticles.PosY[I] - CorrY * Ratio1;
  FParticles.PosX[J] := FParticles.PosX[J] + CorrX * Ratio2;
  FParticles.PosY[J] := FParticles.PosY[J] + CorrY * Ratio2;

  // === Apply impulse (bounce) ===

  // Implicit Verlet velocities
  V1X := FParticles.PosX[I] - FParticles.OldPosX[I];
  V1Y := FParticles.PosY[I] - FParticles.OldPosY[I];
  V2X := FParticles.PosX[J] - FParticles.OldPosX[J];
  V2Y := FParticles.PosY[J] - FParticles.OldPosY[J];

  // Relative velocity along normal
  RelVel := (V2X - V1X) * NormalX + (V2Y - V1Y) * NormalY;

  if RelVel > 0 then
    Exit;

  // Impulse with restitution
  ImpulseScalar := -(1 + FRestitution) * RelVel / TotalInvMass;
  ImpX := NormalX * ImpulseScalar;
  ImpY := NormalY * ImpulseScalar;

  // Modify OldPos for impulse
  FParticles.OldPosX[I] := FParticles.PosX[I] - (V1X - ImpX * InvMass1);
  FParticles.OldPosY[I] := FParticles.PosY[I] - (V1Y - ImpY * InvMass1);
  FParticles.OldPosX[J] := FParticles.PosX[J] - (V2X + ImpX * InvMass2);
  FParticles.OldPosY[J] := FParticles.PosY[J] - (V2Y + ImpY * InvMass2);
end;

procedure TPhyWorld.CollideAABBsSoA(I, J: Integer);
var
  MinX1, MaxX1, MinY1, MaxY1: Single;
  MinX2, MaxX2, MinY2, MaxY2: Single;
  OverlapX, OverlapY: Single;
  NX, NY, Overlap: Single;
  TotalInvMass, Ratio1, Ratio2: Single;
  InvMass1, InvMass2: Single;
  V1X, V1Y, V2X, V2Y: Single;
  RelVel, ImpulseScalar: Single;
  ImpX, ImpY: Single;
begin
  // Compute bounds
  MinX1 := FAABBs.PosX[I] - FAABBs.HalfW[I];
  MaxX1 := FAABBs.PosX[I] + FAABBs.HalfW[I];
  MinY1 := FAABBs.PosY[I] - FAABBs.HalfH[I];
  MaxY1 := FAABBs.PosY[I] + FAABBs.HalfH[I];

  MinX2 := FAABBs.PosX[J] - FAABBs.HalfW[J];
  MaxX2 := FAABBs.PosX[J] + FAABBs.HalfW[J];
  MinY2 := FAABBs.PosY[J] - FAABBs.HalfH[J];
  MaxY2 := FAABBs.PosY[J] + FAABBs.HalfH[J];

  // AABB overlap test
  if (MaxX1 < MinX2) or (MinX1 > MaxX2) or (MaxY1 < MinY2) or (MinY1 > MaxY2) then
    Exit;

  // Compute overlap on each axis
  if FAABBs.PosX[I] < FAABBs.PosX[J] then
    OverlapX := MaxX1 - MinX2
  else
    OverlapX := MaxX2 - MinX1;

  if FAABBs.PosY[I] < FAABBs.PosY[J] then
    OverlapY := MaxY1 - MinY2
  else
    OverlapY := MaxY2 - MinY1;

  // Push along minimum overlap axis
  if OverlapX < OverlapY then
  begin
    Overlap := OverlapX;
    if FAABBs.PosX[I] < FAABBs.PosX[J] then
    begin NX := -1; NY := 0; end
    else
    begin NX := 1; NY := 0; end;
  end
  else
  begin
    Overlap := OverlapY;
    if FAABBs.PosY[I] < FAABBs.PosY[J] then
    begin NX := 0; NY := -1; end
    else
    begin NX := 0; NY := 1; end;
  end;

  InvMass1 := FAABBs.InvMass[I];
  InvMass2 := FAABBs.InvMass[J];
  TotalInvMass := InvMass1 + InvMass2;
  if TotalInvMass < 0.0001 then
    Exit;

  Ratio1 := InvMass1 / TotalInvMass;
  Ratio2 := InvMass2 / TotalInvMass;

  // Separate
  FAABBs.PosX[I] := FAABBs.PosX[I] + NX * Overlap * Ratio1;
  FAABBs.PosY[I] := FAABBs.PosY[I] + NY * Overlap * Ratio1;
  FAABBs.PosX[J] := FAABBs.PosX[J] - NX * Overlap * Ratio2;
  FAABBs.PosY[J] := FAABBs.PosY[J] - NY * Overlap * Ratio2;

  // Verlet velocities
  V1X := FAABBs.PosX[I] - FAABBs.OldPosX[I];
  V1Y := FAABBs.PosY[I] - FAABBs.OldPosY[I];
  V2X := FAABBs.PosX[J] - FAABBs.OldPosX[J];
  V2Y := FAABBs.PosY[J] - FAABBs.OldPosY[J];

  RelVel := (V1X - V2X) * NX + (V1Y - V2Y) * NY;
  if RelVel > 0 then
    Exit;

  ImpulseScalar := -(1 + FRestitution) * RelVel / TotalInvMass;
  ImpX := NX * ImpulseScalar;
  ImpY := NY * ImpulseScalar;

  FAABBs.OldPosX[I] := FAABBs.PosX[I] - (V1X + ImpX * InvMass1);
  FAABBs.OldPosY[I] := FAABBs.PosY[I] - (V1Y + ImpY * InvMass1);
  FAABBs.OldPosX[J] := FAABBs.PosX[J] - (V2X - ImpX * InvMass2);
  FAABBs.OldPosY[J] := FAABBs.PosY[J] - (V2Y - ImpY * InvMass2);
end;

procedure TPhyWorld.CollideParticleAABBSoA(PI, AI: Integer);
var
  PosX, PosY, Rad: Single;
  MinX, MaxX, MinY, MaxY: Single;
  ClosestX, ClosestY: Single;
  DX, DY, DistSq, Dist, Overlap: Single;
  NX, NY, InvDist: Single;
  TotalInvMass, Ratio1, Ratio2: Single;
  InvMassP, InvMassA: Single;
  VPX, VPY, VAX, VAY: Single;
  RelVel, ImpulseScalar, ImpX, ImpY: Single;
begin
  PosX := FParticles.PosX[PI];
  PosY := FParticles.PosY[PI];
  Rad := FParticles.Radius[PI];

  MinX := FAABBs.PosX[AI] - FAABBs.HalfW[AI];
  MaxX := FAABBs.PosX[AI] + FAABBs.HalfW[AI];
  MinY := FAABBs.PosY[AI] - FAABBs.HalfH[AI];
  MaxY := FAABBs.PosY[AI] + FAABBs.HalfH[AI];

  // Closest point on AABB to particle center
  if PosX < MinX then ClosestX := MinX
  else if PosX > MaxX then ClosestX := MaxX
  else ClosestX := PosX;

  if PosY < MinY then ClosestY := MinY
  else if PosY > MaxY then ClosestY := MaxY
  else ClosestY := PosY;

  DX := PosX - ClosestX;
  DY := PosY - ClosestY;
  DistSq := DX * DX + DY * DY;

  if DistSq >= Rad * Rad then
    Exit;

  InvMassP := FParticles.InvMass[PI];
  InvMassA := FAABBs.InvMass[AI];
  TotalInvMass := InvMassP + InvMassA;
  if TotalInvMass < 0.0001 then
    Exit;

  if DistSq < 0.0001 then
  begin
    // Particle inside AABB - push to nearest edge
    DX := PosX - FAABBs.PosX[AI];
    DY := PosY - FAABBs.PosY[AI];
    if Abs(DX) > Abs(DY) then
    begin
      if DX > 0 then begin NX := 1; NY := 0; Overlap := MaxX - PosX + Rad; end
      else begin NX := -1; NY := 0; Overlap := PosX - MinX + Rad; end;
    end
    else
    begin
      if DY > 0 then begin NX := 0; NY := 1; Overlap := MaxY - PosY + Rad; end
      else begin NX := 0; NY := -1; Overlap := PosY - MinY + Rad; end;
    end;
  end
  else
  begin
    InvDist := FastInvSqrt(DistSq);
    Dist := DistSq * InvDist;
    Overlap := Rad - Dist;
    NX := DX * InvDist;
    NY := DY * InvDist;
  end;

  Ratio1 := InvMassP / TotalInvMass;
  Ratio2 := InvMassA / TotalInvMass;

  // Separate
  FParticles.PosX[PI] := PosX + NX * Overlap * Ratio1;
  FParticles.PosY[PI] := PosY + NY * Overlap * Ratio1;
  FAABBs.PosX[AI] := FAABBs.PosX[AI] - NX * Overlap * Ratio2;
  FAABBs.PosY[AI] := FAABBs.PosY[AI] - NY * Overlap * Ratio2;

  // Verlet velocities
  VPX := FParticles.PosX[PI] - FParticles.OldPosX[PI];
  VPY := FParticles.PosY[PI] - FParticles.OldPosY[PI];
  VAX := FAABBs.PosX[AI] - FAABBs.OldPosX[AI];
  VAY := FAABBs.PosY[AI] - FAABBs.OldPosY[AI];

  RelVel := (VPX - VAX) * NX + (VPY - VAY) * NY;
  if RelVel > 0 then
    Exit;

  ImpulseScalar := -(1 + FRestitution) * RelVel / TotalInvMass;
  ImpX := NX * ImpulseScalar;
  ImpY := NY * ImpulseScalar;

  FParticles.OldPosX[PI] := FParticles.PosX[PI] - (VPX + ImpX * InvMassP);
  FParticles.OldPosY[PI] := FParticles.PosY[PI] - (VPY + ImpY * InvMassP);
  FAABBs.OldPosX[AI] := FAABBs.PosX[AI] - (VAX - ImpX * InvMassA);
  FAABBs.OldPosY[AI] := FAABBs.PosY[AI] - (VAY - ImpY * InvMassA);
end;

procedure TPhyWorld.SolveConstraints;
var
  I, P1, P2: Integer;
  DeltaX, DeltaY: Single;
  DistSq, InvDist, Dist, Diff: Single;
  CorrX, CorrY: Single;
  TotalInvMass, Ratio1, Ratio2: Single;
  InvMass1, InvMass2, Stiffness: Single;
begin
  for I := 0 to FConstraintCount - 1 do
  begin
    P1 := FConstraints[I].P1;
    P2 := FConstraints[I].P2;
    Stiffness := FConstraints[I].Stiffness;

    DeltaX := FParticles.PosX[P2] - FParticles.PosX[P1];
    DeltaY := FParticles.PosY[P2] - FParticles.PosY[P1];
    DistSq := DeltaX * DeltaX + DeltaY * DeltaY;

    if DistSq < 0.0001 then
      Continue;

    InvDist := FastInvSqrt(DistSq);
    Dist := DistSq * InvDist;

    Diff := (Dist - FConstraints[I].RestLength) * InvDist;
    CorrX := DeltaX * Diff * 0.5 * Stiffness;
    CorrY := DeltaY * Diff * 0.5 * Stiffness;

    InvMass1 := FParticles.InvMass[P1];
    InvMass2 := FParticles.InvMass[P2];
    TotalInvMass := InvMass1 + InvMass2;
    if TotalInvMass < 0.0001 then
      Continue;

    Ratio1 := InvMass1 / TotalInvMass;
    Ratio2 := InvMass2 / TotalInvMass;

    FParticles.PosX[P1] := FParticles.PosX[P1] + CorrX * Ratio1 * 2;
    FParticles.PosY[P1] := FParticles.PosY[P1] + CorrY * Ratio1 * 2;
    FParticles.PosX[P2] := FParticles.PosX[P2] - CorrX * Ratio2 * 2;
    FParticles.PosY[P2] := FParticles.PosY[P2] - CorrY * Ratio2 * 2;
  end;
end;

procedure TPhyWorld.RebuildBoxGrid;
var
  TotalCells, I, B: Integer;
  Box: PPhyBox;
  MinCX, MinCY, MaxCX, MaxCY, CX, CY, CellIdx: Integer;
begin
  FBoxGridWidth := Trunc(FWorldWidth * FBoxGridInvCellSize) + 1;
  FBoxGridHeight := Trunc(FWorldHeight * FBoxGridInvCellSize) + 1;
  TotalCells := FBoxGridWidth * FBoxGridHeight;

  SetLength(FBoxGrid, TotalCells);

  for I := 0 to TotalCells - 1 do
    FBoxGrid[I].Count := 0;

  for B := 0 to FBoxCount - 1 do
  begin
    Box := @FBoxes[B];

    MinCX := Trunc(Box^.MinX * FBoxGridInvCellSize);
    MinCY := Trunc(Box^.MinY * FBoxGridInvCellSize);
    MaxCX := Trunc(Box^.MaxX * FBoxGridInvCellSize);
    MaxCY := Trunc(Box^.MaxY * FBoxGridInvCellSize);

    if MinCX < 0 then MinCX := 0;
    if MinCY < 0 then MinCY := 0;
    if MaxCX >= FBoxGridWidth then MaxCX := FBoxGridWidth - 1;
    if MaxCY >= FBoxGridHeight then MaxCY := FBoxGridHeight - 1;

    for CY := MinCY to MaxCY do
      for CX := MinCX to MaxCX do
      begin
        CellIdx := CY * FBoxGridWidth + CX;
        if FBoxGrid[CellIdx].Count < BOX_GRID_MAX_PER_CELL then
        begin
          FBoxGrid[CellIdx].Boxes[FBoxGrid[CellIdx].Count] := Box;
          Inc(FBoxGrid[CellIdx].Count);
        end;
      end;
  end;

  FBoxGridDirty := False;
end;

procedure TPhyWorld.CollideParticleBoxSoA(I: Integer; Box: PPhyBox);
var
  PosX, PosY, Rad: Single;
  ClosestX, ClosestY: Single;
  DX, DY, DistSq, Dist, Overlap: Single;
  NX, NY: Single;
  EdgeDistLeft, EdgeDistRight, EdgeDistTop, EdgeDistBottom, MinEdgeDist: Single;
  InvDist: Single;
  VelX, VelY, VelN, ImpulseScalar: Single;
  NewPosX, NewPosY: Single;
begin
  PosX := FParticles.PosX[I];
  PosY := FParticles.PosY[I];
  Rad := FParticles.Radius[I];

  // Find closest point on rectangle
  if PosX < Box^.MinX then
    ClosestX := Box^.MinX
  else if PosX > Box^.MaxX then
    ClosestX := Box^.MaxX
  else
    ClosestX := PosX;

  if PosY < Box^.MinY then
    ClosestY := Box^.MinY
  else if PosY > Box^.MaxY then
    ClosestY := Box^.MaxY
  else
    ClosestY := PosY;

  DX := PosX - ClosestX;
  DY := PosY - ClosestY;
  DistSq := DX * DX + DY * DY;

  if DistSq < Rad * Rad then
  begin
    if DistSq < 0.0001 then
    begin
      // Circle inside - push to nearest edge
      EdgeDistLeft := PosX - Box^.MinX;
      EdgeDistRight := Box^.MaxX - PosX;
      EdgeDistTop := PosY - Box^.MinY;
      EdgeDistBottom := Box^.MaxY - PosY;
      MinEdgeDist := EdgeDistLeft;
      NX := -1; NY := 0;

      if EdgeDistRight < MinEdgeDist then
      begin
        MinEdgeDist := EdgeDistRight;
        NX := 1; NY := 0;
      end;
      if EdgeDistTop < MinEdgeDist then
      begin
        MinEdgeDist := EdgeDistTop;
        NX := 0; NY := -1;
      end;
      if EdgeDistBottom < MinEdgeDist then
      begin
        MinEdgeDist := EdgeDistBottom;
        NX := 0; NY := 1;
      end;

      NewPosX := PosX + NX * (MinEdgeDist + Rad);
      NewPosY := PosY + NY * (MinEdgeDist + Rad);
    end
    else
    begin
      // Normal case
      InvDist := FastInvSqrt(DistSq);
      Dist := DistSq * InvDist;
      Overlap := Rad - Dist;

      NX := DX * InvDist;
      NY := DY * InvDist;

      NewPosX := PosX + NX * Overlap;
      NewPosY := PosY + NY * Overlap;
    end;

    // Calculate velocity BEFORE position correction (from original position)
    VelX := PosX - FParticles.OldPosX[I];
    VelY := PosY - FParticles.OldPosY[I];

    // Apply position correction
    FParticles.PosX[I] := NewPosX;
    FParticles.PosY[I] := NewPosY;

    // Velocity component along normal
    VelN := VelX * NX + VelY * NY;

    // Only reflect if moving into the box
    if VelN < 0 then
    begin
      // Reflect velocity: remove normal component and add reflected component
      // NewVel = Vel - (1 + Rest) * VelN * Normal
      ImpulseScalar := (1 + FRestitution) * VelN;

      // Update OldPos so that NewPos - NewOldPos = reflected velocity
      FParticles.OldPosX[I] := NewPosX - (VelX - ImpulseScalar * NX);
      FParticles.OldPosY[I] := NewPosY - (VelY - ImpulseScalar * NY);
    end
    else
    begin
      // Keep tangent velocity
      FParticles.OldPosX[I] := NewPosX - VelX;
      FParticles.OldPosY[I] := NewPosY - VelY;
    end;
  end;
end;

procedure TPhyWorld.SolveBoxCollisions;
var
  I, K: Integer;
  PosX, PosY, Rad: Single;
  VelX, VelY: Single;
  CellX, CellY, CellIdx: Integer;
  Cell: ^TBoxGridCell;
  BoundsRight, BoundsBottom: Single;
begin
  if FBoxGridDirty then
    RebuildBoxGrid;

  BoundsRight := FWorldWidth;
  BoundsBottom := FWorldHeight;

  for I := 0 to FParticleCount - 1 do
  begin
    if (FParticles.Flags[I] and PHY_FLAG_FIXED) <> 0 then
      Continue;
    if (FParticles.Flags[I] and PHY_FLAG_COLLIDABLE) = 0 then
      Continue;

    PosX := FParticles.PosX[I];
    PosY := FParticles.PosY[I];
    Rad := FParticles.Radius[I];

    // Collision with world bounds - only read OldPos when needed
    // Left wall
    if PosX < Rad then
    begin
      VelX := PosX - FParticles.OldPosX[I];
      PosX := Rad;
      if VelX < 0 then
        FParticles.OldPosX[I] := PosX + VelX * FRestitution
      else
        FParticles.OldPosX[I] := PosX - VelX;
      FParticles.PosX[I] := PosX;
    end
    // Right wall
    else if PosX > BoundsRight - Rad then
    begin
      VelX := PosX - FParticles.OldPosX[I];
      PosX := BoundsRight - Rad;
      if VelX > 0 then
        FParticles.OldPosX[I] := PosX + VelX * FRestitution
      else
        FParticles.OldPosX[I] := PosX - VelX;
      FParticles.PosX[I] := PosX;
    end;

    // Top wall
    if PosY < Rad then
    begin
      VelY := PosY - FParticles.OldPosY[I];
      PosY := Rad;
      if VelY < 0 then
        FParticles.OldPosY[I] := PosY + VelY * FRestitution
      else
        FParticles.OldPosY[I] := PosY - VelY;
      FParticles.PosY[I] := PosY;
    end
    // Bottom wall
    else if PosY > BoundsBottom - Rad then
    begin
      VelY := PosY - FParticles.OldPosY[I];
      PosY := BoundsBottom - Rad;
      if VelY > 0 then
        FParticles.OldPosY[I] := PosY + VelY * FRestitution
      else
        FParticles.OldPosY[I] := PosY - VelY;
      FParticles.PosY[I] := PosY;
    end;

    // Collision with static boxes
    if FBoxCount > 0 then
    begin
      CellX := Trunc(PosX * FBoxGridInvCellSize);
      CellY := Trunc(PosY * FBoxGridInvCellSize);

      if CellX < 0 then CellX := 0;
      if CellY < 0 then CellY := 0;
      if CellX >= FBoxGridWidth then CellX := FBoxGridWidth - 1;
      if CellY >= FBoxGridHeight then CellY := FBoxGridHeight - 1;

      CellIdx := CellY * FBoxGridWidth + CellX;
      Cell := @FBoxGrid[CellIdx];

      for K := 0 to Cell^.Count - 1 do
        CollideParticleBoxSoA(I, Cell^.Boxes[K]);
    end;
  end;
end;

procedure TPhyWorld.SolveAABBBoxCollisions;
var
  I, K: Integer;
  PosX, PosY, HW, HH: Single;
  VelX, VelY: Single;
  CellX, CellY, CellIdx: Integer;
  Cell: ^TBoxGridCell;
  BoundsRight, BoundsBottom: Single;
begin
  if FBoxGridDirty then
    RebuildBoxGrid;

  BoundsRight := FWorldWidth;
  BoundsBottom := FWorldHeight;

  for I := 0 to FAABBCount - 1 do
  begin
    if (FAABBs.Flags[I] and PHY_FLAG_FIXED) <> 0 then
      Continue;
    if (FAABBs.Flags[I] and PHY_FLAG_COLLIDABLE) = 0 then
      Continue;

    PosX := FAABBs.PosX[I];
    PosY := FAABBs.PosY[I];
    HW := FAABBs.HalfW[I];
    HH := FAABBs.HalfH[I];

    // World bounds collision - only read OldPos when needed
    // Left wall
    if PosX - HW < 0 then
    begin
      VelX := PosX - FAABBs.OldPosX[I];
      PosX := HW;
      if VelX < 0 then
        FAABBs.OldPosX[I] := PosX + VelX * FRestitution
      else
        FAABBs.OldPosX[I] := PosX - VelX;
      FAABBs.PosX[I] := PosX;
    end
    // Right wall
    else if PosX + HW > BoundsRight then
    begin
      VelX := PosX - FAABBs.OldPosX[I];
      PosX := BoundsRight - HW;
      if VelX > 0 then
        FAABBs.OldPosX[I] := PosX + VelX * FRestitution
      else
        FAABBs.OldPosX[I] := PosX - VelX;
      FAABBs.PosX[I] := PosX;
    end;

    // Top wall
    if PosY - HH < 0 then
    begin
      VelY := PosY - FAABBs.OldPosY[I];
      PosY := HH;
      if VelY < 0 then
        FAABBs.OldPosY[I] := PosY + VelY * FRestitution
      else
        FAABBs.OldPosY[I] := PosY - VelY;
      FAABBs.PosY[I] := PosY;
    end
    // Bottom wall
    else if PosY + HH > BoundsBottom then
    begin
      VelY := PosY - FAABBs.OldPosY[I];
      PosY := BoundsBottom - HH;
      if VelY > 0 then
        FAABBs.OldPosY[I] := PosY + VelY * FRestitution
      else
        FAABBs.OldPosY[I] := PosY - VelY;
      FAABBs.PosY[I] := PosY;
    end;

    // Static box collision
    if FBoxCount > 0 then
    begin
      CellX := Trunc(PosX * FBoxGridInvCellSize);
      CellY := Trunc(PosY * FBoxGridInvCellSize);

      if CellX < 0 then CellX := 0;
      if CellY < 0 then CellY := 0;
      if CellX >= FBoxGridWidth then CellX := FBoxGridWidth - 1;
      if CellY >= FBoxGridHeight then CellY := FBoxGridHeight - 1;

      CellIdx := CellY * FBoxGridWidth + CellX;
      Cell := @FBoxGrid[CellIdx];

      for K := 0 to Cell^.Count - 1 do
        CollideAABBBoxSoA(I, Cell^.Boxes[K]);
    end;
  end;
end;

procedure TPhyWorld.CollideAABBBoxSoA(I: Integer; Box: PPhyBox);
var
  MinX1, MaxX1, MinY1, MaxY1: Single;
  OverlapX, OverlapY, Overlap: Single;
  NX, NY: Single;
  NewPosX, NewPosY: Single;
  VelX, VelY, VelN: Single;
  ImpulseScalar: Single;
begin
  MinX1 := FAABBs.PosX[I] - FAABBs.HalfW[I];
  MaxX1 := FAABBs.PosX[I] + FAABBs.HalfW[I];
  MinY1 := FAABBs.PosY[I] - FAABBs.HalfH[I];
  MaxY1 := FAABBs.PosY[I] + FAABBs.HalfH[I];

  // AABB overlap test
  if (MaxX1 < Box^.MinX) or (MinX1 > Box^.MaxX) or
     (MaxY1 < Box^.MinY) or (MinY1 > Box^.MaxY) then
    Exit;

  // Compute overlap
  if FAABBs.PosX[I] < (Box^.MinX + Box^.MaxX) * 0.5 then
    OverlapX := MaxX1 - Box^.MinX
  else
    OverlapX := Box^.MaxX - MinX1;

  if FAABBs.PosY[I] < (Box^.MinY + Box^.MaxY) * 0.5 then
    OverlapY := MaxY1 - Box^.MinY
  else
    OverlapY := Box^.MaxY - MinY1;

  // Push along minimum overlap axis
  if OverlapX < OverlapY then
  begin
    Overlap := OverlapX;
    if FAABBs.PosX[I] < (Box^.MinX + Box^.MaxX) * 0.5 then
    begin NX := -1; NY := 0; end
    else
    begin NX := 1; NY := 0; end;
  end
  else
  begin
    Overlap := OverlapY;
    if FAABBs.PosY[I] < (Box^.MinY + Box^.MaxY) * 0.5 then
    begin NX := 0; NY := -1; end
    else
    begin NX := 0; NY := 1; end;
  end;

  // Calculate velocity BEFORE position correction (from original position)
  VelX := FAABBs.PosX[I] - FAABBs.OldPosX[I];
  VelY := FAABBs.PosY[I] - FAABBs.OldPosY[I];

  // Apply position correction
  NewPosX := FAABBs.PosX[I] + NX * Overlap;
  NewPosY := FAABBs.PosY[I] + NY * Overlap;
  FAABBs.PosX[I] := NewPosX;
  FAABBs.PosY[I] := NewPosY;

  // Velocity component along normal
  VelN := VelX * NX + VelY * NY;

  // Only reflect if moving into the box
  if VelN < 0 then
  begin
    // Reflect velocity: remove normal component and add reflected component
    ImpulseScalar := (1 + FRestitution) * VelN;

    // Update OldPos so that NewPos - NewOldPos = reflected velocity
    FAABBs.OldPosX[I] := NewPosX - (VelX - ImpulseScalar * NX);
    FAABBs.OldPosY[I] := NewPosY - (VelY - ImpulseScalar * NY);
  end
  else
  begin
    // Keep tangent velocity
    FAABBs.OldPosX[I] := NewPosX - VelX;
    FAABBs.OldPosY[I] := NewPosY - VelY;
  end;
end;

procedure TPhyWorld.Clear;
begin
  FParticleCount := 0;
  FAABBCount := 0;
  FBoxCount := 0;
  FConstraintCount := 0;
  FRigidBodyCount := 0;
end;

function TPhyWorld.GetParticle(Index: Integer): PPhyParticle;
begin
  // Rebuild temporary structure for compatibility
  FTempParticle.Pos.X := FParticles.PosX[Index];
  FTempParticle.Pos.Y := FParticles.PosY[Index];
  FTempParticle.OldPos.X := FParticles.OldPosX[Index];
  FTempParticle.OldPos.Y := FParticles.OldPosY[Index];
  FTempParticle.Accel.X := FParticles.AccelX[Index];
  FTempParticle.Accel.Y := FParticles.AccelY[Index];
  FTempParticle.Radius := FParticles.Radius[Index];
  FTempParticle.InvMass := FParticles.InvMass[Index];
  FTempParticle.Restitution := FParticles.Restitution[Index];
  FTempParticle.Flags := FParticles.Flags[Index];
  Result := @FTempParticle;
end;

function TPhyWorld.GetBox(Index: Integer): PPhyBox;
begin
  Result := @FBoxes[Index];
end;

// Direct SoA access - faster for rendering
function TPhyWorld.GetPosX(Index: Integer): Single;
begin
  Result := FParticles.PosX[Index];
end;

function TPhyWorld.GetPosY(Index: Integer): Single;
begin
  Result := FParticles.PosY[Index];
end;

function TPhyWorld.GetRadius(Index: Integer): Single;
begin
  Result := FParticles.Radius[Index];
end;

function TPhyWorld.GetOldPosX(Index: Integer): Single;
begin
  Result := FParticles.OldPosX[Index];
end;

function TPhyWorld.GetOldPosY(Index: Integer): Single;
begin
  Result := FParticles.OldPosY[Index];
end;

function TPhyWorld.GetAABB(Index: Integer): PPhyAABB;
begin
  FTempAABB.PosX := FAABBs.PosX[Index];
  FTempAABB.PosY := FAABBs.PosY[Index];
  FTempAABB.OldPosX := FAABBs.OldPosX[Index];
  FTempAABB.OldPosY := FAABBs.OldPosY[Index];
  FTempAABB.AccelX := FAABBs.AccelX[Index];
  FTempAABB.AccelY := FAABBs.AccelY[Index];
  FTempAABB.HalfW := FAABBs.HalfW[Index];
  FTempAABB.HalfH := FAABBs.HalfH[Index];
  FTempAABB.InvMass := FAABBs.InvMass[Index];
  FTempAABB.Restitution := FAABBs.Restitution[Index];
  FTempAABB.Flags := FAABBs.Flags[Index];
  Result := @FTempAABB;
end;

function TPhyWorld.GetAABBPosX(Index: Integer): Single;
begin
  Result := FAABBs.PosX[Index];
end;

function TPhyWorld.GetAABBPosY(Index: Integer): Single;
begin
  Result := FAABBs.PosY[Index];
end;

function TPhyWorld.GetAABBOldPosX(Index: Integer): Single;
begin
  Result := FAABBs.OldPosX[Index];
end;

function TPhyWorld.GetAABBOldPosY(Index: Integer): Single;
begin
  Result := FAABBs.OldPosY[Index];
end;

function TPhyWorld.GetAABBHalfW(Index: Integer): Single;
begin
  Result := FAABBs.HalfW[Index];
end;

function TPhyWorld.GetAABBHalfH(Index: Integer): Single;
begin
  Result := FAABBs.HalfH[Index];
end;

function TPhyWorld.GetConstraint(Index: Integer): PPhyConstraint;
begin
  Result := @FConstraints[Index];
end;

end.
