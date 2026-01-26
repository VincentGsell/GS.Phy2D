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

unit GS.Phy.Types;
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

const
  PHY_FLAG_FIXED     = 1;
  PHY_FLAG_COLLIDABLE = 2;

type
  // Particle: compact record, no class
  TPhyParticle = record
    Pos: TVec2;        // Current position
    OldPos: TVec2;     // Previous position (Verlet)
    Accel: TVec2;      // Accumulated acceleration
    Radius: Single;    // Collision radius
    InvMass: Single;   // 1/mass (0 = fixed)
    Restitution: Single; // Elasticity (0-1)
    Flags: Byte;       // PHY_FLAG_*
  end;
  PPhyParticle = ^TPhyParticle;

  // Distance constraint between 2 particles
  TPhyConstraint = record
    P1, P2: Integer;   // Particle indices
    RestLength: Single;
    Stiffness: Single; // 0-1
  end;
  PPhyConstraint = ^TPhyConstraint;

  // Static box (wall)
  TPhyBox = record
    MinX, MinY, MaxX, MaxY: Single;
    Restitution: Single;
  end;
  PPhyBox = ^TPhyBox;

  // Rigid body: group of particles forming a single object
  TPhyRigidBody = record
    ParticleStart: Integer;  // First particle index
    ParticleCount: Integer;  // Number of particles in this body
    ConstraintStart: Integer; // First constraint index
    ConstraintCount: Integer; // Number of constraints
  end;
  PPhyRigidBody = ^TPhyRigidBody;

function CreateParticle(X, Y, Radius: Single; Fixed: Boolean = False; Mass: Single = 1.0; Restitution: Single = 0.5): TPhyParticle;
function CreateBox(X, Y, Width, Height: Single; Restitution: Single = 0.3): TPhyBox;

implementation

function CreateParticle(X, Y, Radius: Single; Fixed: Boolean; Mass: Single; Restitution: Single): TPhyParticle;
begin
  Result.Pos := Vec2(X, Y);
  Result.OldPos := Vec2(X, Y);
  Result.Accel := Vec2(0, 0);
  Result.Radius := Radius;
  if Fixed or (Mass <= 0) then
    Result.InvMass := 0
  else
    Result.InvMass := 1.0 / Mass;
  Result.Restitution := Restitution;
  Result.Flags := PHY_FLAG_COLLIDABLE;
  if Fixed then
    Result.Flags := Result.Flags or PHY_FLAG_FIXED;
end;

function CreateBox(X, Y, Width, Height: Single; Restitution: Single): TPhyBox;
begin
  Result.MinX := X - Width * 0.5;
  Result.MaxX := X + Width * 0.5;
  Result.MinY := Y - Height * 0.5;
  Result.MaxY := Y + Height * 0.5;
  Result.Restitution := Restitution;
end;

end.
