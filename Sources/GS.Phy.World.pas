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


unit GS.Phy.World;
{$IFDEF FPC}
{$MODE DELPHI}
{$ENDIF}

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
    Boxes: array[0..BOX_GRID_MAX_PER_CELL-1] of PPhyBox;  // Pointeurs directs - elimine l'indirection
    Count: Integer;
  end;

  TPhyWorld = class
  private
    FParticles: array of TPhyParticle;
    FParticleCount: Integer;
    FParticleCapacity: Integer;

    FBoxes: array of TPhyBox;
    FBoxCount: Integer;

    FConstraints: array of TPhyConstraint;
    FConstraintCount: Integer;

    FSpatialHash: TPhySpatialHash;
    FGravity: TVec2;
    FDamping: Single;
    FWorldWidth, FWorldHeight: Single;

    FCollisionIterations: Integer;
    FConstraintIterations: Integer;
    FRestitution: Single;  // Coefficient de rebond global

    // Spatial hash pour boxes statiques
    FBoxGrid: array of TBoxGridCell;
    FBoxGridWidth, FBoxGridHeight: Integer;
    FBoxGridCellSize: Single;
    FBoxGridInvCellSize: Single;
    FBoxGridDirty: Boolean;

    procedure Integrate(DT: Single);
    procedure SolveCollisions;
    procedure SolveConstraints;
    procedure SolveBoxCollisions;
    procedure CollideParticles(I, J: Integer);
    procedure CollideParticlesPtr(P1, P2: PPhyParticle);  // Version optimisee avec pointeurs
    procedure RebuildBoxGrid;
    procedure CollideParticleBox(P: PPhyParticle; Box: PPhyBox);
  public
    constructor Create;
    destructor Destroy; override;

    procedure SetWorldBounds(Width, Height: Single);
    procedure SetGravity(X, Y: Single);

    function AddParticle(X, Y, Radius: Single; Fixed: Boolean = False;
      Mass: Single = 1.0; Restitution: Single = 0.5): Integer;
    function AddBox(X, Y, Width, Height: Single; Restitution: Single = 0.3): Integer;
    function AddConstraint(P1, P2: Integer; Stiffness: Single = 1.0): Integer;

    procedure Step(DT: Single);
    procedure Clear;

    function GetParticle(Index: Integer): PPhyParticle; inline;
    function GetBox(Index: Integer): PPhyBox; inline;

    property ParticleCount: Integer read FParticleCount;
    property BoxCount: Integer read FBoxCount;
    property ConstraintCount: Integer read FConstraintCount;
    property CollisionIterations: Integer read FCollisionIterations write FCollisionIterations;
    property ConstraintIterations: Integer read FConstraintIterations write FConstraintIterations;
    property Damping: Single read FDamping write FDamping;
    property Restitution: Single read FRestitution write FRestitution;  // Coefficient de rebond (0=mou, 1=elastique)
  end;

implementation

const
  INITIAL_CAPACITY = 256;

constructor TPhyWorld.Create;
begin
  inherited;
  FParticleCount := 0;
  FParticleCapacity := INITIAL_CAPACITY;
  SetLength(FParticles, FParticleCapacity);

  FBoxCount := 0;
  SetLength(FBoxes, 16);

  FConstraintCount := 0;
  SetLength(FConstraints, 64);

  FSpatialHash := TPhySpatialHash.Create;
  FGravity := Vec2(0, 500); // Gravite par defaut
  FDamping := 0.99;
  FWorldWidth := 800;
  FWorldHeight := 600;

  FCollisionIterations := 2;
  FConstraintIterations := 1;
  FRestitution := 0.3;  // 30% de rebond par defaut (perte de 70% d'energie)

  FSpatialHash.Setup(FWorldWidth, FWorldHeight, 32);

  // Box grid - cellule de 32 pixels
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

  // Trouver le rayon max pour la taille de cellule
  MaxRadius := 10;
  for I := 0 to FParticleCount - 1 do
    if FParticles[I].Radius > MaxRadius then
      MaxRadius := FParticles[I].Radius;

  FSpatialHash.Setup(Width, Height, MaxRadius * 2);

  // Reconfigurer le box grid
  FBoxGridDirty := True;
end;

procedure TPhyWorld.SetGravity(X, Y: Single);
begin
  FGravity := Vec2(X, Y);
end;

function TPhyWorld.AddParticle(X, Y, Radius: Single; Fixed: Boolean;
  Mass: Single; Restitution: Single): Integer;
begin
  if FParticleCount >= FParticleCapacity then
  begin
    FParticleCapacity := FParticleCapacity * 2;
    SetLength(FParticles, FParticleCapacity);
  end;

  FParticles[FParticleCount] := CreateParticle(X, Y, Radius, Fixed, Mass, Restitution);
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

  // Marquer le box grid comme dirty
  FBoxGridDirty := True;
end;

function TPhyWorld.AddConstraint(P1, P2: Integer; Stiffness: Single): Integer;
var
  Dist: Single;
begin
  if FConstraintCount >= Length(FConstraints) then
    SetLength(FConstraints, Length(FConstraints) * 2);

  Dist := Vec2Dist(FParticles[P1].Pos, FParticles[P2].Pos);

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
  // Integration Verlet
  Integrate(DT);

  // Resoudre contraintes et collisions
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
  P: PPhyParticle;
  Velocity, NewPos: TVec2;
  DT2: Single;
begin
  DT2 := DT * DT;

  for I := 0 to FParticleCount - 1 do
  begin
    P := @FParticles[I];

    // Sauter les particules fixes
    if (P^.Flags and PHY_FLAG_FIXED) <> 0 then
      Continue;

    // Verlet: new_pos = pos + (pos - old_pos) * damping + accel * dt^2
    Velocity := Vec2Sub(P^.Pos, P^.OldPos);
    Velocity := Vec2Mul(Velocity, FDamping);

    // Ajouter gravite
    P^.Accel := Vec2Add(P^.Accel, FGravity);

    NewPos := Vec2Add(P^.Pos, Velocity);
    NewPos := Vec2Add(NewPos, Vec2Mul(P^.Accel, DT2));

    P^.OldPos := P^.Pos;
    P^.Pos := NewPos;

    // Reset acceleration
    P^.Accel := Vec2(0, 0);
  end;
end;

procedure TPhyWorld.SolveCollisions;
var
  I, K, QueryCount: Integer;
  P, P2: PPhyParticle;
begin
  // Rebuild spatial hash avec pointeurs
  FSpatialHash.Clear;
  for I := 0 to FParticleCount - 1 do
  begin
    P := @FParticles[I];
    if (P^.Flags and PHY_FLAG_COLLIDABLE) <> 0 then
      FSpatialHash.Insert(P, P^.Pos.X, P^.Pos.Y, P^.Radius);
  end;

  // Tester collisions via spatial hash - acces direct via pointeurs
  for I := 0 to FParticleCount - 1 do
  begin
    P := @FParticles[I];
    if (P^.Flags and PHY_FLAG_COLLIDABLE) = 0 then
      Continue;

    QueryCount := FSpatialHash.Query(P, P^.Pos.X, P^.Pos.Y, P^.Radius);

    for K := 0 to QueryCount - 1 do
    begin
      P2 := PPhyParticle(FSpatialHash.GetQueryResult(K));
      CollideParticlesPtr(P, P2);  // Nouvelle version avec pointeurs
    end;
  end;
end;

procedure TPhyWorld.CollideParticles(I, J: Integer);
begin
  // Deleguer a la version pointeur
  CollideParticlesPtr(@FParticles[I], @FParticles[J]);
end;

procedure TPhyWorld.CollideParticlesPtr(P1, P2: PPhyParticle);
var
  Delta: TVec2;
  DistSq, Dist, MinDist, Overlap: Single;
  Normal, Correction: TVec2;
  TotalInvMass, Ratio1, Ratio2: Single;
  RelVel, VelAlongNormal: Single;
  V1, V2: TVec2;
  ImpulseScalar: Single;
  Impulse: TVec2;
begin
  Delta := Vec2Sub(P2^.Pos, P1^.Pos);
  DistSq := Vec2LengthSq(Delta);
  MinDist := P1^.Radius + P2^.Radius;

  if DistSq >= MinDist * MinDist then
    Exit; // Pas de collision

  if DistSq < 0.0001 then
  begin
    // Particules au meme endroit - separer aleatoirement
    Delta := Vec2(0.1, 0.1);
    DistSq := Vec2LengthSq(Delta);
  end;

  Dist := Sqrt(DistSq);
  Overlap := MinDist - Dist;
  Normal := Vec2Mul(Delta, 1.0 / Dist);

  // Correction proportionnelle aux masses inverses
  TotalInvMass := P1^.InvMass + P2^.InvMass;
  if TotalInvMass < 0.0001 then
    Exit; // Les deux sont fixes

  Ratio1 := P1^.InvMass / TotalInvMass;
  Ratio2 := P2^.InvMass / TotalInvMass;

  Correction := Vec2Mul(Normal, Overlap);

  // Separer les particules
  P1^.Pos := Vec2Sub(P1^.Pos, Vec2Mul(Correction, Ratio1));
  P2^.Pos := Vec2Add(P2^.Pos, Vec2Mul(Correction, Ratio2));

  // Appliquer la restitution (perte d'energie au rebond)
  // Dans Verlet, la velocite est implicite: V = Pos - OldPos
  V1 := Vec2Sub(P1^.Pos, P1^.OldPos);
  V2 := Vec2Sub(P2^.Pos, P2^.OldPos);

  // Velocite relative le long de la normale
  RelVel := Vec2Dot(Vec2Sub(V2, V1), Normal);

  // Ne rien faire si les particules s'eloignent deja
  if RelVel > 0 then
    Exit;

  // Calculer l'impulsion avec restitution
  ImpulseScalar := -(1 + FRestitution) * RelVel / TotalInvMass;
  Impulse := Vec2Mul(Normal, ImpulseScalar);

  // Modifier OldPos pour simuler le changement de velocite
  // NewV1 = V1 - Impulse * InvMass1  =>  NewOldPos1 = Pos - NewV1
  P1^.OldPos := Vec2Sub(P1^.Pos, Vec2Sub(V1, Vec2Mul(Impulse, P1^.InvMass)));
  P2^.OldPos := Vec2Sub(P2^.Pos, Vec2Add(V2, Vec2Mul(Impulse, P2^.InvMass)));
end;

procedure TPhyWorld.SolveConstraints;
var
  I: Integer;
  C: PPhyConstraint;
  P1, P2: PPhyParticle;
  Delta: TVec2;
  Dist, Diff: Single;
  Correction: TVec2;
  TotalInvMass, Ratio1, Ratio2: Single;
begin
  for I := 0 to FConstraintCount - 1 do
  begin
    C := @FConstraints[I];
    P1 := @FParticles[C^.P1];
    P2 := @FParticles[C^.P2];

    Delta := Vec2Sub(P2^.Pos, P1^.Pos);
    Dist := Vec2Length(Delta);

    if Dist < 0.0001 then
      Continue;

    Diff := (Dist - C^.RestLength) / Dist;
    Correction := Vec2Mul(Delta, Diff * 0.5 * C^.Stiffness);

    TotalInvMass := P1^.InvMass + P2^.InvMass;
    if TotalInvMass < 0.0001 then
      Continue;

    Ratio1 := P1^.InvMass / TotalInvMass;
    Ratio2 := P2^.InvMass / TotalInvMass;

    P1^.Pos := Vec2Add(P1^.Pos, Vec2Mul(Correction, Ratio1 * 2));
    P2^.Pos := Vec2Sub(P2^.Pos, Vec2Mul(Correction, Ratio2 * 2));
  end;
end;

procedure TPhyWorld.RebuildBoxGrid;
var
  TotalCells, I, B: Integer;
  Box: PPhyBox;
  MinCX, MinCY, MaxCX, MaxCY, CX, CY, CellIdx: Integer;
begin
  // Calculer dimensions de la grille
  FBoxGridWidth := Trunc(FWorldWidth * FBoxGridInvCellSize) + 1;
  FBoxGridHeight := Trunc(FWorldHeight * FBoxGridInvCellSize) + 1;
  TotalCells := FBoxGridWidth * FBoxGridHeight;

  // Allouer/reallouer la grille
  SetLength(FBoxGrid, TotalCells);

  // Reset tous les compteurs
  for I := 0 to TotalCells - 1 do
    FBoxGrid[I].Count := 0;

  // Inserer chaque box dans toutes les cellules qu'elle couvre
  for B := 0 to FBoxCount - 1 do
  begin
    Box := @FBoxes[B];

    // Calculer les cellules couvertes par la box
    MinCX := Trunc(Box^.MinX * FBoxGridInvCellSize);
    MinCY := Trunc(Box^.MinY * FBoxGridInvCellSize);
    MaxCX := Trunc(Box^.MaxX * FBoxGridInvCellSize);
    MaxCY := Trunc(Box^.MaxY * FBoxGridInvCellSize);

    // Clamp aux limites de la grille
    if MinCX < 0 then MinCX := 0;
    if MinCY < 0 then MinCY := 0;
    if MaxCX >= FBoxGridWidth then MaxCX := FBoxGridWidth - 1;
    if MaxCY >= FBoxGridHeight then MaxCY := FBoxGridHeight - 1;

    // Inserer dans toutes les cellules couvertes
    for CY := MinCY to MaxCY do
      for CX := MinCX to MaxCX do
      begin
        CellIdx := CY * FBoxGridWidth + CX;
        if FBoxGrid[CellIdx].Count < BOX_GRID_MAX_PER_CELL then
        begin
          FBoxGrid[CellIdx].Boxes[FBoxGrid[CellIdx].Count] := Box;  // Stocker le pointeur directement
          Inc(FBoxGrid[CellIdx].Count);
        end;
        // Si overflow, on ignore (rare en pratique)
      end;
  end;

  FBoxGridDirty := False;
end;

procedure TPhyWorld.CollideParticleBox(P: PPhyParticle; Box: PPhyBox);
var
  ClosestX, ClosestY: Single;
  DX, DY, DistSq, Dist, Overlap: Single;
  NX, NY: Single;
  EdgeDistLeft, EdgeDistRight, EdgeDistTop, EdgeDistBottom, MinEdgeDist: Single;
begin
  // Trouver le point le plus proche sur le rectangle
  if P^.Pos.X < Box^.MinX then
    ClosestX := Box^.MinX
  else if P^.Pos.X > Box^.MaxX then
    ClosestX := Box^.MaxX
  else
    ClosestX := P^.Pos.X;

  if P^.Pos.Y < Box^.MinY then
    ClosestY := Box^.MinY
  else if P^.Pos.Y > Box^.MaxY then
    ClosestY := Box^.MaxY
  else
    ClosestY := P^.Pos.Y;

  // Vecteur du point le plus proche vers le centre du cercle
  DX := P^.Pos.X - ClosestX;
  DY := P^.Pos.Y - ClosestY;
  DistSq := DX * DX + DY * DY;

  // Collision si distance < rayon
  if DistSq < P^.Radius * P^.Radius then
  begin
    // Cas special: cercle completement a l'interieur du rectangle
    if DistSq < 0.0001 then
    begin
      // Pousser vers le bord le plus proche
      EdgeDistLeft := P^.Pos.X - Box^.MinX;
      EdgeDistRight := Box^.MaxX - P^.Pos.X;
      EdgeDistTop := P^.Pos.Y - Box^.MinY;
      EdgeDistBottom := Box^.MaxY - P^.Pos.Y;
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

      P^.Pos.X := P^.Pos.X + NX * (MinEdgeDist + P^.Radius);
      P^.Pos.Y := P^.Pos.Y + NY * (MinEdgeDist + P^.Radius);
    end
    else
    begin
      // Cas normal: pousser le cercle hors du rectangle
      Dist := Sqrt(DistSq);
      Overlap := P^.Radius - Dist;

      // Normaliser le vecteur de direction
      NX := DX / Dist;
      NY := DY / Dist;

      // Deplacer le cercle exactement au bord
      P^.Pos.X := P^.Pos.X + NX * Overlap;
      P^.Pos.Y := P^.Pos.Y + NY * Overlap;
    end;
  end;
end;

procedure TPhyWorld.SolveBoxCollisions;
var
  I, K: Integer;
  P: PPhyParticle;
  CellX, CellY, CellIdx: Integer;
  Cell: ^TBoxGridCell;
begin
  // Rebuild box grid si necessaire
  if FBoxGridDirty then
    RebuildBoxGrid;

  for I := 0 to FParticleCount - 1 do
  begin
    P := @FParticles[I];
    if (P^.Flags and PHY_FLAG_FIXED) <> 0 then
      Continue;
    if (P^.Flags and PHY_FLAG_COLLIDABLE) = 0 then
      Continue;

    // Collision avec les bords du monde (containeur principal)
    if P^.Pos.X < P^.Radius then
      P^.Pos.X := P^.Radius;
    if P^.Pos.X > FWorldWidth - P^.Radius then
      P^.Pos.X := FWorldWidth - P^.Radius;
    if P^.Pos.Y < P^.Radius then
      P^.Pos.Y := P^.Radius;
    if P^.Pos.Y > FWorldHeight - P^.Radius then
      P^.Pos.Y := FWorldHeight - P^.Radius;

    // Collision avec les boxes via spatial hash
    if FBoxCount > 0 then
    begin
      // Trouver la cellule de la particule
      CellX := Trunc(P^.Pos.X * FBoxGridInvCellSize);
      CellY := Trunc(P^.Pos.Y * FBoxGridInvCellSize);

      // Clamp
      if CellX < 0 then CellX := 0;
      if CellY < 0 then CellY := 0;
      if CellX >= FBoxGridWidth then CellX := FBoxGridWidth - 1;
      if CellY >= FBoxGridHeight then CellY := FBoxGridHeight - 1;

      CellIdx := CellY * FBoxGridWidth + CellX;
      Cell := @FBoxGrid[CellIdx];

      // Tester collision avec les boxes de cette cellule - acces direct via pointeur
      for K := 0 to Cell^.Count - 1 do
        CollideParticleBox(P, Cell^.Boxes[K]);  // Plus besoin de @FBoxes[B]
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
  Result := @FParticles[Index];
end;

function TPhyWorld.GetBox(Index: Integer): PPhyBox;
begin
  Result := @FBoxes[Index];
end;

end.
