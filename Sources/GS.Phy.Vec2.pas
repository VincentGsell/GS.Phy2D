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

unit GS.Phy.Vec2;
{$IFDEF FPC}
{$MODE DELPHI}
{$ENDIF}

// Compiler optimizations
{$O+}  // Optimizations enabled
{$R-}  // Range checking disabled
{$Q-}  // Overflow checking disabled

interface

type
  TVec2 = record
    X, Y: Single;
  end;
  PVec2 = ^TVec2;

  // Vec3 for circle definitions (X, Y, Radius)
  TVec3 = record
    X, Y, Z: Single;
  end;
  PVec3 = ^TVec3;

function Vec2(X, Y: Single): TVec2; inline;
function Vec3(X, Y, Z: Single): TVec3; inline;
function Vec2Add(const A, B: TVec2): TVec2; inline;
function Vec2Sub(const A, B: TVec2): TVec2; inline;
function Vec2Mul(const V: TVec2; S: Single): TVec2; inline;
function Vec2Dot(const A, B: TVec2): Single; inline;
function Vec2LengthSq(const V: TVec2): Single; inline;
function Vec2Length(const V: TVec2): Single; inline;
function Vec2Normalize(const V: TVec2): TVec2; inline;
function Vec2NormalizeFast(const V: TVec2): TVec2; inline;  // Fast version
function Vec2Dist(const A, B: TVec2): Single; inline;
function Vec2DistSq(const A, B: TVec2): Single; inline;
function FastInvSqrt(X: Single): Single; inline;  // Quake III algorithm

implementation

function Vec2(X, Y: Single): TVec2;
begin
  Result.X := X;
  Result.Y := Y;
end;

function Vec3(X, Y, Z: Single): TVec3;
begin
  Result.X := X;
  Result.Y := Y;
  Result.Z := Z;
end;

function Vec2Add(const A, B: TVec2): TVec2;
begin
  Result.X := A.X + B.X;
  Result.Y := A.Y + B.Y;
end;

function Vec2Sub(const A, B: TVec2): TVec2;
begin
  Result.X := A.X - B.X;
  Result.Y := A.Y - B.Y;
end;

function Vec2Mul(const V: TVec2; S: Single): TVec2;
begin
  Result.X := V.X * S;
  Result.Y := V.Y * S;
end;

function Vec2Dot(const A, B: TVec2): Single;
begin
  Result := A.X * B.X + A.Y * B.Y;
end;

function Vec2LengthSq(const V: TVec2): Single;
begin
  Result := V.X * V.X + V.Y * V.Y;
end;

function Vec2Length(const V: TVec2): Single;
begin
  Result := Sqrt(V.X * V.X + V.Y * V.Y);
end;

function Vec2Normalize(const V: TVec2): TVec2;
var
  Len: Single;
begin
  Len := Vec2Length(V);
  if Len > 0.0001 then
  begin
    Result.X := V.X / Len;
    Result.Y := V.Y / Len;
  end
  else
  begin
    Result.X := 0;
    Result.Y := 0;
  end;
end;

function Vec2Dist(const A, B: TVec2): Single;
begin
  Result := Sqrt(Sqr(B.X - A.X) + Sqr(B.Y - A.Y));
end;

function Vec2DistSq(const A, B: TVec2): Single;
begin
  Result := Sqr(B.X - A.X) + Sqr(B.Y - A.Y);
end;

// Fast Inverse Square Root - Quake III algorithm (John Carmack)
// ~1% precision but much faster than 1/Sqrt(x)
function FastInvSqrt(X: Single): Single;
var
  I: Integer;
  X2, Y: Single;
begin
  X2 := X * 0.5;
  Y := X;
  I := PInteger(@Y)^;               // Reinterpret float as int
  I := $5F3759DF - (I shr 1);       // Magic number (initial approximation)
  Y := PSingle(@I)^;                // Reinterpret int as float
  Y := Y * (1.5 - (X2 * Y * Y));    // One Newton-Raphson iteration
  Result := Y;
end;

// Fast normalization using FastInvSqrt
function Vec2NormalizeFast(const V: TVec2): TVec2;
var
  LenSq, InvLen: Single;
begin
  LenSq := V.X * V.X + V.Y * V.Y;
  if LenSq > 0.0001 then
  begin
    InvLen := FastInvSqrt(LenSq);
    Result.X := V.X * InvLen;
    Result.Y := V.Y * InvLen;
  end
  else
  begin
    Result.X := 0;
    Result.Y := 0;
  end;
end;

end.
