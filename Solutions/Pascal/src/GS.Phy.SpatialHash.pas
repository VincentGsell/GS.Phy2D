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

// Compiler optimizations
{$O+}  // Optimizations enabled
{$R-}  // Range checking disabled
{$Q-}  // Overflow checking disabled

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

    // Grid: each cell contains pointers to particles
    FCells: array of array of Pointer;  // Direct pointers - eliminates indirection
    FCellCounts: array of Integer;
    FCellCapacity: Integer;

    // For queries - stores pointers
    FQueryResult: array of Pointer;
    FQueryCount: Integer;

    function GetCellIndex(X, Y: Single; out CellX, CellY: Integer): Boolean; inline;
  public
    constructor Create;
    destructor Destroy; override;

    procedure Setup(WorldWidth, WorldHeight, CellSize: Single);
    procedure Clear;
    procedure Insert(Ptr: Pointer; X, Y, Radius: Single);
    function Query(SelfPtr: Pointer; X, Y, Radius: Single): Integer; // Returns result count
    function GetQueryResult(I: Integer): Pointer; inline;

    property CellSize: Single read FCellSize;
  end;

implementation

const
  INITIAL_CELL_CAPACITY = 16;
  MAX_CELL_CAPACITY = 64;   // Max capacity per cell (avoids reallocations)
  MAX_QUERY_RESULTS = 256;  // Fixed capacity to avoid allocations

constructor TPhySpatialHash.Create;
begin
  inherited;
  FCellSize := 32;
  FInvCellSize := 1 / FCellSize;
  FCellCapacity := INITIAL_CELL_CAPACITY;
  // Pre-allocate result buffer once
  SetLength(FQueryResult, MAX_QUERY_RESULTS);
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

  // Allocate grid once with fixed capacity
  SetLength(FCells, TotalCells, MAX_CELL_CAPACITY);
  SetLength(FCellCounts, TotalCells);

  // Fill with zeros
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
  // Compute cells covered by the circle
  GetCellIndex(X - Radius, Y - Radius, MinCX, MinCY);
  GetCellIndex(X + Radius, Y + Radius, MaxCX, MaxCY);

  // Clamp
  if MinCX < 0 then MinCX := 0;
  if MinCY < 0 then MinCY := 0;
  if MaxCX >= FGridWidth then MaxCX := FGridWidth - 1;
  if MaxCY >= FGridHeight then MaxCY := FGridHeight - 1;

  // Insert into all covered cells
  for CY := MinCY to MaxCY do
    for CX := MinCX to MaxCX do
    begin
      CellIdx := CY * FGridWidth + CX;
      Count := FCellCounts[CellIdx];

      // Fixed capacity - ignore if full (rare with good cell size)
      if Count < MAX_CELL_CAPACITY then
      begin
        FCells[CellIdx][Count] := Ptr;  // Store pointer directly
        Inc(FCellCounts[CellIdx]);
      end;
    end;
end;

function TPhySpatialHash.Query(SelfPtr: Pointer; X, Y, Radius: Single): Integer;
var
  MinCX, MinCY, MaxCX, MaxCY: Integer;
  CX, CY, CellIdx, I, Count: Integer;
  Ptr: Pointer;
begin
  FQueryCount := 0;

  // Compute cells to query
  GetCellIndex(X - Radius, Y - Radius, MinCX, MinCY);
  GetCellIndex(X + Radius, Y + Radius, MaxCX, MaxCY);

  // Clamp
  if MinCX < 0 then MinCX := 0;
  if MinCY < 0 then MinCY := 0;
  if MaxCX >= FGridWidth then MaxCX := FGridWidth - 1;
  if MaxCY >= FGridHeight then MaxCY := FGridHeight - 1;

  // Iterate cells
  for CY := MinCY to MaxCY do
    for CX := MinCX to MaxCX do
    begin
      CellIdx := CY * FGridWidth + CX;
      Count := FCellCounts[CellIdx];

      for I := 0 to Count - 1 do
      begin
        Ptr := FCells[CellIdx][I];

        // Don't return self, avoid duplicates (larger pointer only)
        if NativeUInt(Ptr) > NativeUInt(SelfPtr) then
        begin
          // Pre-allocated buffer - ignore if full (rare in practice)
          if FQueryCount < MAX_QUERY_RESULTS then
          begin
            FQueryResult[FQueryCount] := Ptr;
            Inc(FQueryCount);
          end;
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
