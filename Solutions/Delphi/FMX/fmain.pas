unit fmain;

interface

uses
  System.SysUtils, System.Types, System.UITypes, System.Classes,
  System.Diagnostics,
  FMX.Types, FMX.Controls, FMX.Forms, FMX.Graphics, FMX.Dialogs, FMX.SpinBox,
  FMX.Controls.Presentation, FMX.StdCtrls, FMX.Layouts,
  GS.Phy.Vec2, GS.Phy.Types, GS.Phy.AABB, GS.Phy.World,
  GS.Phy.Renderer, GS.Phy.Renderer.FMX,
  FMX.Edit, FMX.EditBox, FMX.ActnList, FMX.Objects;

type
  TFormGSPhy = class(TForm)
    TimerPhysics: TTimer;
    Selection1: TSelection;
    Rectangle1: TRectangle;
    ButtonAddBalls: TButton;
    ButtonAddAABBs: TButton;
    Label1: TLabel;
    LabelFPS: TLabel;
    LabelRestitution: TLabel;
    TrackBarRestitution: TTrackBar;
    TrackBarDamping: TTrackBar;
    LabelDamping: TLabel;
    LabelIterations: TLabel;
    SpinBoxIterations: TSpinBox;
    ButtonAddBoxes: TButton;
    LabelBallCount: TLabel;
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure FormPaint(Sender: TObject; Canvas: TCanvas; const ARect: TRectF);
    procedure FormResize(Sender: TObject);
    procedure TimerPhysicsTimer(Sender: TObject);
    procedure ButtonAddBallsClick(Sender: TObject);
    procedure ButtonAddBoxesClick(Sender: TObject);
    procedure ButtonAddAABBsClick(Sender: TObject);
    procedure SpinBoxIterationsChange(Sender: TObject);
    procedure TrackBarDampingChange(Sender: TObject);
    procedure TrackBarRestitutionChange(Sender: TObject);
  private
    FWorld: TPhyWorld;
    FRenderer: TPhyRendererFMX;
    FFrameCreated: Boolean;
    FStopwatch: TStopwatch;
    FFrameCount: Integer;
    FLastFPSUpdate: Int64;
    procedure CreateFrame;
    procedure UpdateFrame;
    procedure SpawnBalls(Count: Integer);
    procedure SpawnBoxes(Count: Integer);
    procedure SpawnAABBs(Count: Integer);
    procedure RenderWorld;
  end;

var
  FormGSPhy: TFormGSPhy;

implementation

{$R *.fmx}

const
  WALL_THICKNESS = 20;
  BALL_RADIUS_MIN = 5;
  BALL_RADIUS_MAX = 15;

procedure TFormGSPhy.FormCreate(Sender: TObject);
begin
  FFrameCreated := False;

  FWorld := TPhyWorld.Create;
  FWorld.SetGravity(0, 800);
  FWorld.Damping := 0.99;
  FWorld.CollisionIterations := 2;

  FRenderer := TPhyRendererFMX.Create;

  SpinBoxIterations.Value := FWorld.CollisionIterations;

  FStopwatch := TStopwatch.StartNew;
  FFrameCount := 0;
  FLastFPSUpdate := 0;
end;

procedure TFormGSPhy.FormDestroy(Sender: TObject);
begin
  FRenderer.Free;
  FWorld.Free;
end;

procedure TFormGSPhy.FormResize(Sender: TObject);
begin
  if not FFrameCreated then
    CreateFrame
  else
    UpdateFrame;
end;

procedure TFormGSPhy.CreateFrame;
var
  W, H: Single;
begin
  W := ClientWidth;
  H := ClientHeight;

  FWorld.Clear;
  FWorld.SetWorldBounds(W, H);
  FFrameCreated := True;
end;

procedure TFormGSPhy.UpdateFrame;
var
  W, H: Single;
begin
  W := ClientWidth;
  H := ClientHeight;
  FWorld.SetWorldBounds(W, H);
end;

procedure TFormGSPhy.SpawnBalls(Count: Integer);
var
  I: Integer;
  X, Y, Radius: Single;
  SpawnLeft, SpawnRight, SpawnTop, SpawnBottom: Single;
begin
  SpawnLeft := BALL_RADIUS_MAX + 10;
  SpawnRight := ClientWidth - BALL_RADIUS_MAX - 10;
  SpawnTop := BALL_RADIUS_MAX + 10;
  SpawnBottom := ClientHeight / 3;

  for I := 0 to Count - 1 do
  begin
    X := SpawnLeft + Random * (SpawnRight - SpawnLeft);
    Y := SpawnTop + Random * (SpawnBottom - SpawnTop);
    Radius := BALL_RADIUS_MIN + Random * (BALL_RADIUS_MAX - BALL_RADIUS_MIN);

    FWorld.AddParticle(X, Y, Radius, False, 1.0, 0.5);
  end;
end;

procedure TFormGSPhy.SpawnBoxes(Count: Integer);
const
  BOX_MIN_SIZE = 15;
  BOX_MAX_SIZE = 40;
var
  I: Integer;
  X, Y, BoxW, BoxH: Single;
  SpawnLeft, SpawnRight, SpawnTop, SpawnBottom: Single;
begin
  SpawnLeft := BOX_MAX_SIZE + 20;
  SpawnRight := ClientWidth - BOX_MAX_SIZE - 20;
  SpawnTop := ClientHeight / 3;
  SpawnBottom := ClientHeight - BOX_MAX_SIZE - 20;

  for I := 0 to Count - 1 do
  begin
    X := SpawnLeft + Random * (SpawnRight - SpawnLeft);
    Y := SpawnTop + Random * (SpawnBottom - SpawnTop);
    BoxW := BOX_MIN_SIZE + Random * (BOX_MAX_SIZE - BOX_MIN_SIZE);
    BoxH := BOX_MIN_SIZE + Random * (BOX_MAX_SIZE - BOX_MIN_SIZE);

    FWorld.AddBox(X, Y, BoxW, BoxH);
  end;
end;

procedure TFormGSPhy.TimerPhysicsTimer(Sender: TObject);
var
  ElapsedMs: Int64;
  TotalDynamic: Integer;
begin
  TimerPhysics.Enabled := False;
  try
    FWorld.Step(1.0 / 60.0);

    Inc(FFrameCount);
    ElapsedMs := FStopwatch.ElapsedMilliseconds;
    if ElapsedMs - FLastFPSUpdate >= 500 then
    begin
      LabelFPS.Text := Format('FPS: %.1f', [FFrameCount * 1000 / (ElapsedMs - FLastFPSUpdate)]);
      FFrameCount := 0;
      FLastFPSUpdate := ElapsedMs;

      // Update counters
      TotalDynamic := FWorld.ParticleCount + FWorld.AABBCount;
      Label1.Text := Format('Particles: %d', [TotalDynamic]);
      LabelBallCount.Text := Format('Balls: %d / AABB: %d', [FWorld.ParticleCount, FWorld.AABBCount]);
    end;

    Invalidate;
  finally
    TimerPhysics.Enabled := True;
  end;
end;

procedure TFormGSPhy.ButtonAddBallsClick(Sender: TObject);
begin
  SpawnBalls(50);
end;

procedure TFormGSPhy.ButtonAddBoxesClick(Sender: TObject);
begin
  SpawnBoxes(50);
end;

procedure TFormGSPhy.ButtonAddAABBsClick(Sender: TObject);
begin
  SpawnAABBs(50);
end;

procedure TFormGSPhy.SpawnAABBs(Count: Integer);
const
  AABB_MIN_SIZE = 10;
  AABB_MAX_SIZE = 30;
var
  I: Integer;
  X, Y, W, H: Single;
  SpawnLeft, SpawnRight, SpawnTop, SpawnBottom: Single;
begin
  SpawnLeft := AABB_MAX_SIZE + 10;
  SpawnRight := ClientWidth - AABB_MAX_SIZE - 10;
  SpawnTop := AABB_MAX_SIZE + 10;
  SpawnBottom := ClientHeight / 3;

  for I := 0 to Count - 1 do
  begin
    X := SpawnLeft + Random * (SpawnRight - SpawnLeft);
    Y := SpawnTop + Random * (SpawnBottom - SpawnTop);
    W := AABB_MIN_SIZE + Random * (AABB_MAX_SIZE - AABB_MIN_SIZE);
    H := AABB_MIN_SIZE + Random * (AABB_MAX_SIZE - AABB_MIN_SIZE);

    FWorld.AddAABB(X, Y, W, H, False, 1.0, 0.5);
  end;
end;

procedure TFormGSPhy.SpinBoxIterationsChange(Sender: TObject);
begin
  FWorld.CollisionIterations := Trunc(SpinBoxIterations.Value);
  LabelIterations.Text := Format('Iter: %d', [FWorld.CollisionIterations]);
end;

procedure TFormGSPhy.TrackBarDampingChange(Sender: TObject);
var
  DampValue: Single;
begin
  DampValue := TrackBarDamping.Value / 100;
  FWorld.Damping := DampValue;
  LabelDamping.Text := Format('Damp: %d%%', [Trunc(TrackBarDamping.Value)]);
end;

procedure TFormGSPhy.TrackBarRestitutionChange(Sender: TObject);
var
  RestValue: Single;
begin
  RestValue := TrackBarRestitution.Value / 100;
  FWorld.Restitution := RestValue;
  LabelRestitution.Text := Format('Rest: %d%%', [Trunc(TrackBarRestitution.Value)]);
end;

procedure TFormGSPhy.FormPaint(Sender: TObject; Canvas: TCanvas;
  const ARect: TRectF);
begin
  FRenderer.SetCanvas(Canvas);
  FRenderer.SetSize(ClientWidth, ClientHeight);
  FRenderer.BeginRender;
  try
    FRenderer.Clear(TPhyColors.White);
    RenderWorld;
  finally
    FRenderer.EndRender;
  end;
end;

procedure TFormGSPhy.RenderWorld;
var
  I: Integer;
  Box: PPhyBox;
  PosX, PosY, OldPosX, OldPosY, Radius: Single;
  HalfW, HalfH: Single;
  VelX, VelY: Single;
  ObjColor: TPhyColor;
  W, H: Single;
begin
  W := FRenderer.Width;
  H := FRenderer.Height;

  // World borders
  FRenderer.FillRect(0, 0, W, 2, TPhyColors.LightGray);
  FRenderer.FillRect(0, H - 2, W, H, TPhyColors.LightGray);
  FRenderer.FillRect(0, 0, 2, H, TPhyColors.LightGray);
  FRenderer.FillRect(W - 2, 0, W, H, TPhyColors.LightGray);

  // Static boxes
  for I := 0 to FWorld.BoxCount - 1 do
  begin
    Box := FWorld.GetBox(I);
    FRenderer.FillRect(Box^.MinX, Box^.MinY, Box^.MaxX, Box^.MaxY, TPhyColors.Gray);
  end;

  // Particles with velocity color
  for I := 0 to FWorld.ParticleCount - 1 do
  begin
    PosX := FWorld.GetPosX(I);
    PosY := FWorld.GetPosY(I);
    OldPosX := FWorld.GetOldPosX(I);
    OldPosY := FWorld.GetOldPosY(I);
    Radius := FWorld.GetRadius(I);

    VelX := PosX - OldPosX;
    VelY := PosY - OldPosY;
    ObjColor := FRenderer.VelocityToColor(VelX, VelY);

    FRenderer.FillCircle(PosX, PosY, Radius, ObjColor);
  end;

  // Dynamic AABBs with velocity color
  for I := 0 to FWorld.AABBCount - 1 do
  begin
    PosX := FWorld.GetAABBPosX(I);
    PosY := FWorld.GetAABBPosY(I);
    OldPosX := FWorld.GetAABBOldPosX(I);
    OldPosY := FWorld.GetAABBOldPosY(I);
    HalfW := FWorld.GetAABBHalfW(I);
    HalfH := FWorld.GetAABBHalfH(I);

    VelX := PosX - OldPosX;
    VelY := PosY - OldPosY;
    ObjColor := FRenderer.VelocityToColor(VelX, VelY);

    FRenderer.FillRect(PosX - HalfW, PosY - HalfH, PosX + HalfW, PosY + HalfH, ObjColor);
  end;
end;

end.
