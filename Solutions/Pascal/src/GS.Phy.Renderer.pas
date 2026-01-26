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
//20260110 - Created. Abstract renderer base class.

unit GS.Phy.Renderer;
{$IFDEF FPC}
{$MODE DELPHI}
{$ENDIF}

interface

type
  // 32-bit ARGB color
  TPhyColor = Cardinal;

  // Abstract renderer class
  TPhyRenderer = class abstract
  private
    FWidth, FHeight: Single;
    FMaxSpeed: Single;  // For velocity color calculation
  protected
    // Abstract methods to be implemented by derived classes
    procedure DoBeginRender; virtual; abstract;
    procedure DoEndRender; virtual; abstract;
    procedure DoClear(Color: TPhyColor); virtual; abstract;
    procedure DoFillCircle(X, Y, Radius: Single; Color: TPhyColor); virtual; abstract;
    procedure DoDrawCircle(X, Y, Radius: Single; Color: TPhyColor; Thickness: Single); virtual; abstract;
    procedure DoFillRect(MinX, MinY, MaxX, MaxY: Single; Color: TPhyColor); virtual; abstract;
    procedure DoDrawRect(MinX, MinY, MaxX, MaxY: Single; Color: TPhyColor; Thickness: Single); virtual; abstract;
    procedure DoFillRotatedRect(CX, CY, HalfW, HalfH, Angle: Single; Color: TPhyColor); virtual; abstract;
    procedure DoDrawRotatedRect(CX, CY, HalfW, HalfH, Angle: Single; Color: TPhyColor; Thickness: Single); virtual; abstract;
    procedure DoDrawLine(X1, Y1, X2, Y2: Single; Color: TPhyColor; Thickness: Single); virtual; abstract;
  public
    constructor Create;
    destructor Destroy; override;

    // Public interface
    procedure BeginRender;
    procedure EndRender;
    procedure Clear(Color: TPhyColor);
    procedure SetSize(Width, Height: Single);

    // Drawing primitives
    procedure FillCircle(X, Y, Radius: Single; Color: TPhyColor);
    procedure DrawCircle(X, Y, Radius: Single; Color: TPhyColor; Thickness: Single = 1.0);
    procedure FillRect(MinX, MinY, MaxX, MaxY: Single; Color: TPhyColor);
    procedure DrawRect(MinX, MinY, MaxX, MaxY: Single; Color: TPhyColor; Thickness: Single = 1.0);
    procedure FillRotatedRect(CX, CY, HalfW, HalfH, Angle: Single; Color: TPhyColor);
    procedure DrawRotatedRect(CX, CY, HalfW, HalfH, Angle: Single; Color: TPhyColor; Thickness: Single = 1.0);
    procedure DrawLine(X1, Y1, X2, Y2: Single; Color: TPhyColor; Thickness: Single = 1.0);

    // Utility: computes particle color based on velocity
    function VelocityToColor(VelX, VelY: Single): TPhyColor;

    property Width: Single read FWidth;
    property Height: Single read FHeight;
    property MaxSpeed: Single read FMaxSpeed write FMaxSpeed;
  end;

  // Predefined colors (ARGB format)
  TPhyColors = class
  public const
    White     = TPhyColor($FFFFFFFF);
    Black     = TPhyColor($FF000000);
    Red       = TPhyColor($FFFF0000);
    Green     = TPhyColor($FF00FF00);
    Blue      = TPhyColor($FF0000FF);
    Yellow    = TPhyColor($FFFFFF00);
    Cyan      = TPhyColor($FF00FFFF);
    Magenta   = TPhyColor($FFFF00FF);
    Gray      = TPhyColor($FF808080);
    LightGray = TPhyColor($FFD3D3D3);
    DarkGray  = TPhyColor($FF4040FF);
  end;

implementation

constructor TPhyRenderer.Create;
begin
  inherited;
  FWidth := 800;
  FHeight := 600;
  FMaxSpeed := 15.0;
end;

destructor TPhyRenderer.Destroy;
begin
  inherited;
end;

procedure TPhyRenderer.BeginRender;
begin
  DoBeginRender;
end;

procedure TPhyRenderer.EndRender;
begin
  DoEndRender;
end;

procedure TPhyRenderer.Clear(Color: TPhyColor);
begin
  DoClear(Color);
end;

procedure TPhyRenderer.SetSize(Width, Height: Single);
begin
  FWidth := Width;
  FHeight := Height;
end;

procedure TPhyRenderer.FillCircle(X, Y, Radius: Single; Color: TPhyColor);
begin
  DoFillCircle(X, Y, Radius, Color);
end;

procedure TPhyRenderer.DrawCircle(X, Y, Radius: Single; Color: TPhyColor; Thickness: Single);
begin
  DoDrawCircle(X, Y, Radius, Color, Thickness);
end;

procedure TPhyRenderer.FillRect(MinX, MinY, MaxX, MaxY: Single; Color: TPhyColor);
begin
  DoFillRect(MinX, MinY, MaxX, MaxY, Color);
end;

procedure TPhyRenderer.DrawRect(MinX, MinY, MaxX, MaxY: Single; Color: TPhyColor; Thickness: Single);
begin
  DoDrawRect(MinX, MinY, MaxX, MaxY, Color, Thickness);
end;

procedure TPhyRenderer.FillRotatedRect(CX, CY, HalfW, HalfH, Angle: Single; Color: TPhyColor);
begin
  DoFillRotatedRect(CX, CY, HalfW, HalfH, Angle, Color);
end;

procedure TPhyRenderer.DrawRotatedRect(CX, CY, HalfW, HalfH, Angle: Single; Color: TPhyColor; Thickness: Single);
begin
  DoDrawRotatedRect(CX, CY, HalfW, HalfH, Angle, Color, Thickness);
end;

procedure TPhyRenderer.DrawLine(X1, Y1, X2, Y2: Single; Color: TPhyColor; Thickness: Single);
begin
  DoDrawLine(X1, Y1, X2, Y2, Color, Thickness);
end;

function TPhyRenderer.VelocityToColor(VelX, VelY: Single): TPhyColor;
var
  Speed, SpeedNorm: Single;
  R, G, B: Byte;
begin
  Speed := Sqrt(VelX * VelX + VelY * VelY);
  SpeedNorm := Speed / FMaxSpeed;
  if SpeedNorm > 1.0 then SpeedNorm := 1.0;

  // Gradient: Blue -> Cyan -> Green -> Yellow -> Red
  if SpeedNorm < 0.25 then
  begin
    R := 0;
    G := Round(SpeedNorm * 4 * 255);
    B := 255;
  end
  else if SpeedNorm < 0.5 then
  begin
    R := 0;
    G := 255;
    B := Round((1 - (SpeedNorm - 0.25) * 4) * 255);
  end
  else if SpeedNorm < 0.75 then
  begin
    R := Round((SpeedNorm - 0.5) * 4 * 255);
    G := 255;
    B := 0;
  end
  else
  begin
    R := 255;
    G := Round((1 - (SpeedNorm - 0.75) * 4) * 255);
    B := 0;
  end;

  Result := $FF000000 or (Cardinal(R) shl 16) or (Cardinal(G) shl 8) or Cardinal(B);
end;

end.
