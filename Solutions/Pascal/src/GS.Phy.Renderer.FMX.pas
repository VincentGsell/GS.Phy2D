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
//20260110 - Created. FMX Canvas renderer implementation.

unit GS.Phy.Renderer.FMX;

interface

uses
  System.Types,
  System.UITypes,
  System.Math,
  System.Math.Vectors,
  FMX.Graphics,
  GS.Phy.Renderer,
  GS.Phy.World;

type
  TPhyRendererFMX = class(TPhyRenderer)
  private
    FCanvas: TCanvas;
    FSceneActive: Boolean;
  protected
    procedure DoBeginRender; override;
    procedure DoEndRender; override;
    procedure DoClear(Color: TPhyColor); override;
    procedure DoFillCircle(X, Y, Radius: Single; Color: TPhyColor); override;
    procedure DoDrawCircle(X, Y, Radius: Single; Color: TPhyColor; Thickness: Single); override;
    procedure DoFillRect(MinX, MinY, MaxX, MaxY: Single; Color: TPhyColor); override;
    procedure DoDrawRect(MinX, MinY, MaxX, MaxY: Single; Color: TPhyColor; Thickness: Single); override;
    procedure DoFillRotatedRect(CX, CY, HalfW, HalfH, Angle: Single; Color: TPhyColor); override;
    procedure DoDrawRotatedRect(CX, CY, HalfW, HalfH, Angle: Single; Color: TPhyColor; Thickness: Single); override;
    procedure DoDrawLine(X1, Y1, X2, Y2: Single; Color: TPhyColor; Thickness: Single); override;
  public
    constructor Create;
    destructor Destroy; override;

    // Set target FMX canvas
    procedure SetCanvas(Canvas: TCanvas);

    property Canvas: TCanvas read FCanvas;
  end;

implementation

constructor TPhyRendererFMX.Create;
begin
  inherited;
  FCanvas := nil;
  FSceneActive := False;
end;

destructor TPhyRendererFMX.Destroy;
begin
  inherited;
end;

procedure TPhyRendererFMX.SetCanvas(Canvas: TCanvas);
begin
  FCanvas := Canvas;
end;

procedure TPhyRendererFMX.DoBeginRender;
begin
  if (FCanvas <> nil) and FCanvas.BeginScene then
    FSceneActive := True
  else
    FSceneActive := False;
end;

procedure TPhyRendererFMX.DoEndRender;
begin
  if FSceneActive and (FCanvas <> nil) then
    FCanvas.EndScene;
  FSceneActive := False;
end;

procedure TPhyRendererFMX.DoClear(Color: TPhyColor);
begin
  if FSceneActive and (FCanvas <> nil) then
    FCanvas.Clear(TAlphaColor(Color));
end;

procedure TPhyRendererFMX.DoFillCircle(X, Y, Radius: Single; Color: TPhyColor);
var
  Rect: TRectF;
begin
  if FSceneActive and (FCanvas <> nil) then
  begin
    FCanvas.Fill.Color := TAlphaColor(Color);
    Rect := RectF(X - Radius, Y - Radius, X + Radius, Y + Radius);
    FCanvas.FillEllipse(Rect, 1.0);
  end;
end;

procedure TPhyRendererFMX.DoDrawCircle(X, Y, Radius: Single; Color: TPhyColor; Thickness: Single);
var
  Rect: TRectF;
begin
  if FSceneActive and (FCanvas <> nil) then
  begin
    FCanvas.Stroke.Color := TAlphaColor(Color);
    FCanvas.Stroke.Thickness := Thickness;
    Rect := RectF(X - Radius, Y - Radius, X + Radius, Y + Radius);
    FCanvas.DrawEllipse(Rect, 1.0);
  end;
end;

procedure TPhyRendererFMX.DoFillRect(MinX, MinY, MaxX, MaxY: Single; Color: TPhyColor);
var
  Rect: TRectF;
begin
  if FSceneActive and (FCanvas <> nil) then
  begin
    FCanvas.Fill.Color := TAlphaColor(Color);
    Rect := RectF(MinX, MinY, MaxX, MaxY);
    FCanvas.FillRect(Rect, 0, 0, [], 1.0);
  end;
end;

procedure TPhyRendererFMX.DoDrawRect(MinX, MinY, MaxX, MaxY: Single; Color: TPhyColor; Thickness: Single);
var
  Rect: TRectF;
begin
  if FSceneActive and (FCanvas <> nil) then
  begin
    FCanvas.Stroke.Color := TAlphaColor(Color);
    FCanvas.Stroke.Thickness := Thickness;
    Rect := RectF(MinX, MinY, MaxX, MaxY);
    FCanvas.DrawRect(Rect, 0, 0, [], 1.0);
  end;
end;

procedure TPhyRendererFMX.DoFillRotatedRect(CX, CY, HalfW, HalfH, Angle: Single; Color: TPhyColor);
var
  SavedMatrix: TMatrix;
  Rect: TRectF;
begin
  if FSceneActive and (FCanvas <> nil) then
  begin
    // Save current transformation
    SavedMatrix := FCanvas.Matrix;

    // Apply rotation around center
    FCanvas.SetMatrix(
      TMatrix.CreateTranslation(-CX, -CY) *
      TMatrix.CreateRotation(Angle) *
      TMatrix.CreateTranslation(CX, CY) *
      SavedMatrix
    );

    // Draw rectangle at center
    FCanvas.Fill.Color := TAlphaColor(Color);
    Rect := RectF(CX - HalfW, CY - HalfH, CX + HalfW, CY + HalfH);
    FCanvas.FillRect(Rect, 0, 0, [], 1.0);

    // Restore transformation
    FCanvas.SetMatrix(SavedMatrix);
  end;
end;

procedure TPhyRendererFMX.DoDrawRotatedRect(CX, CY, HalfW, HalfH, Angle: Single; Color: TPhyColor; Thickness: Single);
var
  SavedMatrix: TMatrix;
  Rect: TRectF;
begin
  if FSceneActive and (FCanvas <> nil) then
  begin
    SavedMatrix := FCanvas.Matrix;

    FCanvas.SetMatrix(
      TMatrix.CreateTranslation(-CX, -CY) *
      TMatrix.CreateRotation(Angle) *
      TMatrix.CreateTranslation(CX, CY) *
      SavedMatrix
    );

    FCanvas.Stroke.Color := TAlphaColor(Color);
    FCanvas.Stroke.Thickness := Thickness;
    Rect := RectF(CX - HalfW, CY - HalfH, CX + HalfW, CY + HalfH);
    FCanvas.DrawRect(Rect, 0, 0, [], 1.0);

    FCanvas.SetMatrix(SavedMatrix);
  end;
end;

procedure TPhyRendererFMX.DoDrawLine(X1, Y1, X2, Y2: Single; Color: TPhyColor; Thickness: Single);
begin
  if FSceneActive and (FCanvas <> nil) then
  begin
    FCanvas.Stroke.Kind := TBrushKind.Solid;
    FCanvas.Stroke.Color := TAlphaColor(Color);
    FCanvas.Stroke.Thickness := Thickness;
    FCanvas.DrawLine(PointF(X1, Y1), PointF(X2, Y2), 1.0);
  end;
end;

end.
