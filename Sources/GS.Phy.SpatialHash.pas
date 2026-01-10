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

unit GS.Phy.SpatialHash;
{$IFDEF FPC}
{$MODE DELPHI}
{$ENDIF}

interface

uses
  GS.Phy.Vec2;

type
  TPhySpatialHash = class
  private
    FCellSize: Single;
    FInvCellSize: Single;
    FGridWidth: Integer;
    FGridHeight: Integer;
    FOffsetX, FOffsetY: Single;

    // Grille : chaque cellule contient des pointeurs vers les particules
    FCells: array of array of Pointer;  // Pointeurs directs - elimine l'indirection
    FCellCounts: array of Integer;
    FCellCapacity: Integer;

    // Pour les queries - stocke des pointeurs
    FQueryResult: array of Pointer;
    FQueryCount: Integer;

    function GetCellIndex(X, Y: Single; out CellX, CellY: Integer): Boolean; inline;
  public
    constructor Create;
    destructor Destroy; override;

    procedure Setup(WorldWidth, WorldHeight, CellSize: Single);
    procedure Clear;
    procedure Insert(Ptr: Pointer; X, Y, Radius: Single);
    function Query(SelfPtr: Pointer; X, Y, Radius: Single): Integer; // Retourne le nombre de resultats
    function GetQueryResult(I: Integer): Pointer; inline;

    property CellSize: Single read FCellSize;
  end;

implementation

const
  INITIAL_CELL_CAPACITY = 16;

constructor TPhySpatialHash.Create;
begin
  inherited;
  FCellSize := 32;
  FInvCellSize := 1 / FCellSize;
  FCellCapacity := INITIAL_CELL_CAPACITY;
end;

destructor TPhySpatialHash.Destroy;
begin
  inherited;
end;

procedure TPhySpatialHash.Setup(WorldWidth, WorldHeight, CellSize: Single);
var
  TotalCells, I: Integer;
begin
  FCellSize := CellSize;
  if FCellSize < 1 then FCellSize := 1;
  FInvCellSize := 1 / FCellSize;

  FGridWidth := Trunc(WorldWidth * FInvCellSize) + 1;
  FGridHeight := Trunc(WorldHeight * FInvCellSize) + 1;
  FOffsetX := 0;
  FOffsetY := 0;

  TotalCells := FGridWidth * FGridHeight;

  // Allouer la grille
  SetLength(FCells, TotalCells, FCellCapacity);
  SetLength(FCellCounts, TotalCells);

  // Remplir a zero
  for I := 0 to TotalCells - 1 do
    FCellCounts[I] := 0;
end;

procedure TPhySpatialHash.Clear;
var
  I: Integer;
begin
  for I := 0 to Length(FCellCounts) - 1 do
    FCellCounts[I] := 0;
end;

function TPhySpatialHash.GetCellIndex(X, Y: Single; out CellX, CellY: Integer): Boolean;
begin
  CellX := Trunc((X - FOffsetX) * FInvCellSize);
  CellY := Trunc((Y - FOffsetY) * FInvCellSize);
  Result := (CellX >= 0) and (CellX < FGridWidth) and
            (CellY >= 0) and (CellY < FGridHeight);
end;

procedure TPhySpatialHash.Insert(Ptr: Pointer; X, Y, Radius: Single);
var
  MinCX, MinCY, MaxCX, MaxCY: Integer;
  CX, CY, CellIdx, Count: Integer;
begin
  // Calculer les cellules couvertes par le cercle
  GetCellIndex(X - Radius, Y - Radius, MinCX, MinCY);
  GetCellIndex(X + Radius, Y + Radius, MaxCX, MaxCY);

  // Clamp
  if MinCX < 0 then MinCX := 0;
  if MinCY < 0 then MinCY := 0;
  if MaxCX >= FGridWidth then MaxCX := FGridWidth - 1;
  if MaxCY >= FGridHeight then MaxCY := FGridHeight - 1;

  // Inserer dans toutes les cellules couvertes
  for CY := MinCY to MaxCY do
    for CX := MinCX to MaxCX do
    begin
      CellIdx := CY * FGridWidth + CX;
      Count := FCellCounts[CellIdx];

      // Agrandir si necessaire
      if Count >= Length(FCells[CellIdx]) then
        SetLength(FCells[CellIdx], Length(FCells[CellIdx]) * 2);

      FCells[CellIdx][Count] := Ptr;  // Stocker le pointeur directement
      Inc(FCellCounts[CellIdx]);
    end;
end;

function TPhySpatialHash.Query(SelfPtr: Pointer; X, Y, Radius: Single): Integer;
var
  MinCX, MinCY, MaxCX, MaxCY: Integer;
  CX, CY, CellIdx, I, Count: Integer;
  Ptr: Pointer;
begin
  FQueryCount := 0;

  // Calculer les cellules a interroger
  GetCellIndex(X - Radius, Y - Radius, MinCX, MinCY);
  GetCellIndex(X + Radius, Y + Radius, MaxCX, MaxCY);

  // Clamp
  if MinCX < 0 then MinCX := 0;
  if MinCY < 0 then MinCY := 0;
  if MaxCX >= FGridWidth then MaxCX := FGridWidth - 1;
  if MaxCY >= FGridHeight then MaxCY := FGridHeight - 1;

  // Parcourir les cellules
  for CY := MinCY to MaxCY do
    for CX := MinCX to MaxCX do
    begin
      CellIdx := CY * FGridWidth + CX;
      Count := FCellCounts[CellIdx];

      for I := 0 to Count - 1 do
      begin
        Ptr := FCells[CellIdx][I];

        // Ne pas retourner soi-meme, et eviter les doublons (pointeur plus grand)
        if NativeUInt(Ptr) > NativeUInt(SelfPtr) then
        begin
          if FQueryCount >= Length(FQueryResult) then
            SetLength(FQueryResult, Length(FQueryResult) + 64);
          FQueryResult[FQueryCount] := Ptr;
          Inc(FQueryCount);
        end;
      end;
    end;

  Result := FQueryCount;
end;

function TPhySpatialHash.GetQueryResult(I: Integer): Pointer;
begin
  Result := FQueryResult[I];
end;

end.
