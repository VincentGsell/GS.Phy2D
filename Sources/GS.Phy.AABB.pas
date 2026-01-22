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
//20260111 - Created. Dynamic AABB object (no rotation).

unit GS.Phy.AABB;
{$IFDEF FPC}
{$MODE DELPHI}
{$ENDIF}

{$O+}
{$R-}
{$Q-}

interface

uses
  GS.Phy.Vec2;

const
  PHY_FLAG_FIXED      = 1;
  PHY_FLAG_COLLIDABLE = 2;

type
  // Dynamic AABB: axis-aligned bounding box without rotation
  TPhyAABB = record
    PosX, PosY: Single;         // Center position
    OldPosX, OldPosY: Single;   // Previous position (Verlet)
    AccelX, AccelY: Single;     // Accumulated acceleration
    HalfW, HalfH: Single;       // Half-extents
    InvMass: Single;            // 1/mass (0 = fixed)
    Restitution: Single;
    Flags: Byte;
  end;
  PPhyAABB = ^TPhyAABB;

  // SoA layout for cache-friendly access
  TAABBSoA = record
    PosX: array of Single;
    PosY: array of Single;
    OldPosX: array of Single;
    OldPosY: array of Single;
    AccelX: array of Single;
    AccelY: array of Single;
    HalfW: array of Single;
    HalfH: array of Single;
    InvMass: array of Single;
    Restitution: array of Single;
    Flags: array of Byte;
  end;

function CreateAABB(X, Y, Width, Height: Single; Fixed: Boolean = False;
  Mass: Single = 1.0; Restitution: Single = 0.5): TPhyAABB;

implementation

function CreateAABB(X, Y, Width, Height: Single; Fixed: Boolean;
  Mass: Single; Restitution: Single): TPhyAABB;
begin
  Result.PosX := X;
  Result.PosY := Y;
  Result.OldPosX := X;
  Result.OldPosY := Y;
  Result.AccelX := 0;
  Result.AccelY := 0;
  Result.HalfW := Width * 0.5;
  Result.HalfH := Height * 0.5;
  if Fixed or (Mass <= 0) then
    Result.InvMass := 0
  else
    Result.InvMass := 1.0 / Mass;
  Result.Restitution := Restitution;
  Result.Flags := PHY_FLAG_COLLIDABLE;
  if Fixed then
    Result.Flags := Result.Flags or PHY_FLAG_FIXED;
end;

end.
