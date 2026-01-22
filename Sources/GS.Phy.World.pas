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

// Optimisations de compilation
{$O+}  // Optimisations activees
{$R-}  // Range checking desactive
{$Q-}  // Overflow checking desactive

interface

uses
  GS.Phy.Vec2,
  GS.Phy.Types,
  GS.Phy.SpatialHash;

const
  BOX_GRID_MAX_PER_CELL = 8;

type
  // Spatial hash pour boxes statiques - array fixe de pointeurs par cellule
  TBoxGridCell = record
    Boxes: array[0..BOX_GRID_MAX_PER_CELL-1] of PPhyBox;
    Count: Integer;
  end;

  // Structure SoA pour les particules - acces cache-friendly
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
  end;

  TPhyWorld = class
  private
    // SoA pour les particules
    FParticles: TParticleSoA;
    FParticleCount: Integer;
    FParticleCapacity: Integer;

    // Pour compatibilite avec GetParticle - cache temporaire
    FTempParticle: TPhyParticle;

    FBoxes: TArray<TPhyBox>;
    FBoxCount: Integer;

    FConstraints: TArray<TPhyConstraint>;
    FConstraintCount: Integer;

    FSpatialHash: TPhySpatialHash;
    FGravity: TVec2;
    FDamping: Single;
    FWorldWidth, FWorldHeight: Single;

    FCollisionIterations: Integer;
    FConstraintIterations: Integer;
    FRestitution: Single;

    // Spatial hash pour boxes statiques
    FBoxGrid: array of TBoxGridCell;
    FBoxGridWidth, FBoxGridHeight: Integer;
    FBoxGridCellSize: Single;
    FBoxGridInvCellSize: Single;
    FBoxGridDirty: Boolean;
    FNeedToSetWorldBounds: Boolean;
    FNeedToSetWorldBoundW, FNeedToSetWorldBoundH: Single;

    procedure GrowParticleArrays;
    procedure Integrate(DT: Single);
    procedure SolveCollisions;
    procedure SolveConstraints;
    procedure SolveBoxCollisions;
    procedure CollideParticlesSoA(I, J: Integer);
    procedure CollideParticleBoxSoA(I: Integer; Box: PPhyBox);
    procedure InternalSetWorldBounds(Width, Height: Single);
  public
    constructor Create;
    destructor Destroy; override;
    procedure RebuildBoxGrid;
    procedure NeedRebuildBoxGrid;

    procedure SetWorldBounds(Width, Height: Single);
    procedure SetGravity(X, Y: Single);

    function AddParticle(X, Y, Radius: Single; Fixed: Boolean = False;
      Mass: Single = 1.0; Restitution: Single = 0.5): Integer;
    function AddBox(X, Y, Width, Height: Single; Restitution: Single = 0.3): Integer;
    function AddConstraint(P1, P2: Integer; Stiffness: Single = 1.0): Integer;

    procedure Step(DT: Single);
    procedure Clear;

    function GetParticle(Index: Integer): PPhyParticle;
    function GetBox(Index: Integer): PPhyBox; inline;

    // Acces direct SoA pour le rendu (plus rapide)
    function GetPosX(Index: Integer): Single; inline;
    function GetPosY(Index: Integer): Single; inline;
    function GetRadius(Index: Integer): Single; inline;
    function GetOldPosX(Index: Integer): Single; inline;
    function GetOldPosY(Index: Integer): Single; inline;

    property ParticleCount: Integer read FParticleCount;
    property BoxCount: Integer read FBoxCount;
    property ConstraintCount: Integer read FConstraintCount;
    property CollisionIterations: Integer read FCollisionIterations write FCollisionIterations;
    property ConstraintIterations: Integer read FConstraintIterations write FConstraintIterations;
    property Damping: Single read FDamping write FDamping;
    property Restitution: Single read FRestitution write FRestitution;
    property Boxes: TArray<TPhyBox> read FBoxes write FBoxes;
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
end;

constructor TPhyWorld.Create;
begin
  inherited;
  FParticleCount := 0;
  FParticleCapacity := INITIAL_CAPACITY;

  // Allouer les tableaux SoA
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

  FBoxCount := 0;
  SetLength(FBoxes, 16);

  FConstraintCount := 0;
  SetLength(FConstraints, 64);

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
begin
  FNeedToSetWorldBounds := True;
  FNeedToSetWorldBoundW := Width;
  FNeedToSetWorldBoundH := Height;
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

  Result := FParticleCount;
  Inc(FParticleCount);
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

procedure TPhyWorld.Step(DT: Single);
var
  Iter: Integer;
begin
  if FNeedToSetWorldBounds then
    InternalSetWorldBounds(FNeedToSetWorldBoundW, FNeedToSetWorldBoundH);
  Integrate(DT);

  for Iter := 0 to FCollisionIterations - 1 do
  begin
    SolveConstraints;
    SolveCollisions;
    SolveBoxCollisions;
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

  // Boucle optimisee SoA - acces sequentiel en memoire
  for I := 0 to FParticleCount - 1 do
  begin
    // Sauter les particules fixes
    if (FParticles.Flags[I] and PHY_FLAG_FIXED) <> 0 then
      Continue;

    // Verlet: new_pos = pos + (pos - old_pos) * damping + accel * dt^2
    VelX := (FParticles.PosX[I] - FParticles.OldPosX[I]) * FDamping;
    VelY := (FParticles.PosY[I] - FParticles.OldPosY[I]) * FDamping;

    // Ajouter gravite a l'acceleration
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

procedure TPhyWorld.InternalSetWorldBounds(Width, Height: Single);
var
  MaxRadius: Single;
  I: Integer;
begin
  FNeedToSetWorldBounds := False;
  FWorldWidth := Width;
  FWorldHeight := Height;

  MaxRadius := 10;
  for I := 0 to FParticleCount - 1 do
    if FParticles.Radius[I] > MaxRadius then
      MaxRadius := FParticles.Radius[I];

  FSpatialHash.Setup(Width, Height, MaxRadius * 2);
  FBoxGridDirty := True;
end;

procedure TPhyWorld.NeedRebuildBoxGrid;
begin
  FBoxGridDirty := True;
end;

procedure TPhyWorld.SolveCollisions;
var
  I, J, K, QueryCount: Integer;
  PosX, PosY, Rad: Single;
begin
  // Rebuild spatial hash - on stocke maintenant les indices
  FSpatialHash.Clear;
  for I := 0 to FParticleCount - 1 do
  begin
    if (FParticles.Flags[I] and PHY_FLAG_COLLIDABLE) <> 0 then
      FSpatialHash.Insert(Pointer(NativeUInt(I)), FParticles.PosX[I], FParticles.PosY[I], FParticles.Radius[I]);
  end;

  // Tester collisions via spatial hash
  for I := 0 to FParticleCount - 1 do
  begin
    if (FParticles.Flags[I] and PHY_FLAG_COLLIDABLE) = 0 then
      Continue;

    PosX := FParticles.PosX[I];
    PosY := FParticles.PosY[I];
    Rad := FParticles.Radius[I];

    QueryCount := FSpatialHash.Query(Pointer(NativeUInt(I)), PosX, PosY, Rad);

    for K := 0 to QueryCount - 1 do
    begin
      J := Integer(NativeUInt(FSpatialHash.GetQueryResult(K)));
      CollideParticlesSoA(I, J);
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
begin
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

  // Separer les particules
  FParticles.PosX[I] := FParticles.PosX[I] - CorrX * Ratio1;
  FParticles.PosY[I] := FParticles.PosY[I] - CorrY * Ratio1;
  FParticles.PosX[J] := FParticles.PosX[J] + CorrX * Ratio2;
  FParticles.PosY[J] := FParticles.PosY[J] + CorrY * Ratio2;

  // Velocites implicites Verlet
  V1X := FParticles.PosX[I] - FParticles.OldPosX[I];
  V1Y := FParticles.PosY[I] - FParticles.OldPosY[I];
  V2X := FParticles.PosX[J] - FParticles.OldPosX[J];
  V2Y := FParticles.PosY[J] - FParticles.OldPosY[J];

  // Velocite relative le long de la normale
  RelVel := (V2X - V1X) * NormalX + (V2Y - V1Y) * NormalY;

  if RelVel > 0 then
    Exit;

  // Impulsion avec restitution
  ImpulseScalar := -(1 + FRestitution) * RelVel / TotalInvMass;
  ImpX := NormalX * ImpulseScalar;
  ImpY := NormalY * ImpulseScalar;

  // Modifier OldPos
  FParticles.OldPosX[I] := FParticles.PosX[I] - (V1X - ImpX * InvMass1);
  FParticles.OldPosY[I] := FParticles.PosY[I] - (V1Y - ImpY * InvMass1);
  FParticles.OldPosX[J] := FParticles.PosX[J] - (V2X + ImpX * InvMass2);
  FParticles.OldPosY[J] := FParticles.PosY[J] - (V2Y + ImpY * InvMass2);
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
begin
  PosX := FParticles.PosX[I];
  PosY := FParticles.PosY[I];
  Rad := FParticles.Radius[I];

  // Trouver le point le plus proche sur le rectangle
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
      // Cercle a l'interieur - pousser vers le bord le plus proche
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

      FParticles.PosX[I] := PosX + NX * (MinEdgeDist + Rad);
      FParticles.PosY[I] := PosY + NY * (MinEdgeDist + Rad);
    end
    else
    begin
      // Cas normal
      InvDist := FastInvSqrt(DistSq);
      Dist := DistSq * InvDist;
      Overlap := Rad - Dist;

      NX := DX * InvDist;
      NY := DY * InvDist;

      FParticles.PosX[I] := PosX + NX * Overlap;
      FParticles.PosY[I] := PosY + NY * Overlap;
    end;
  end;
end;

procedure TPhyWorld.SolveBoxCollisions;
var
  I, K: Integer;
  PosX, PosY, Rad: Single;
  CellX, CellY, CellIdx: Integer;
  Cell: ^TBoxGridCell;
begin
  if FBoxGridDirty then
    RebuildBoxGrid;

  for I := 0 to FParticleCount - 1 do
  begin
    if (FParticles.Flags[I] and PHY_FLAG_FIXED) <> 0 then
      Continue;
    if (FParticles.Flags[I] and PHY_FLAG_COLLIDABLE) = 0 then
      Continue;

    PosX := FParticles.PosX[I];
    PosY := FParticles.PosY[I];
    Rad := FParticles.Radius[I];

    // Collision avec les bords du monde
    if PosX < Rad then
      PosX := Rad;
    if PosX > FWorldWidth - Rad then
      PosX := FWorldWidth - Rad;
    if PosY < Rad then
      PosY := Rad;
    if PosY > FWorldHeight - Rad then
      PosY := FWorldHeight - Rad;

    FParticles.PosX[I] := PosX;
    FParticles.PosY[I] := PosY;

    // Collision avec les boxes
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

procedure TPhyWorld.Clear;
begin
  FParticleCount := 0;
  FBoxCount := 0;
  FConstraintCount := 0;
end;

function TPhyWorld.GetParticle(Index: Integer): PPhyParticle;
begin
  // Reconstruit une structure temporaire pour compatibilite
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

// Acces direct SoA - plus rapide pour le rendu
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

end.
