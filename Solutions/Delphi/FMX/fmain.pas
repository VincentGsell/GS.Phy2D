unit fmain;

interface

uses
  System.SysUtils, System.Types, System.UITypes, System.Classes,
  System.Diagnostics,
  FMX.Types, FMX.Controls, FMX.Forms, FMX.Graphics, FMX.Dialogs, FMX.SpinBox,
  FMX.Controls.Presentation, FMX.StdCtrls, FMX.Layouts,
  GS.Phy.Vec2, GS.Phy.Types, GS.Phy.World, FMX.Edit, FMX.EditBox,
  FMX.ActnList;

type
  TFormGSPhy = class(TForm)
    TimerPhysics: TTimer;
    PanelControls: TPanel;
    ButtonAddBalls: TButton;
    ButtonAddBoxes: TButton;
    LabelBallCount: TLabel;
    LabelFPS: TLabel;
    LabelIterations: TLabel;
    SpinBoxIterations: TSpinBox;
    LabelDamping: TLabel;
    TrackBarDamping: TTrackBar;
    LabelRestitution: TLabel;
    TrackBarRestitution: TTrackBar;
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure FormPaint(Sender: TObject; Canvas: TCanvas; const ARect: TRectF);
    procedure FormResize(Sender: TObject);
    procedure TimerPhysicsTimer(Sender: TObject);
    procedure ButtonAddBallsClick(Sender: TObject);
    procedure ButtonAddBoxesClick(Sender: TObject);
    procedure SpinBoxIterationsChange(Sender: TObject);
    procedure TrackBarDampingChange(Sender: TObject);
    procedure TrackBarRestitutionChange(Sender: TObject);
  private
    FWorld: TPhyWorld;
    FFrameCreated: Boolean;
    FStopwatch: TStopwatch;
    FFrameCount: Integer;
    FLastFPSUpdate: Int64;
    FBallCount: Integer;
    procedure CreateFrame;
    procedure UpdateFrame;
    procedure SpawnBalls(Count: Integer);
    procedure SpawnBoxes(Count: Integer);
    procedure RenderWorld(Canvas: TCanvas);
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
  FBallCount := 0;

  FWorld := TPhyWorld.Create;
  FWorld.SetGravity(0, 800);
  FWorld.Damping := 0.99;
  FWorld.CollisionIterations := 2;

  SpinBoxIterations.Value := FWorld.CollisionIterations;

  FStopwatch := TStopwatch.StartNew;
  FFrameCount := 0;
  FLastFPSUpdate := 0;
end;

procedure TFormGSPhy.FormDestroy(Sender: TObject);
begin
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
  FBallCount := 0;

  FWorld.SetWorldBounds(W, H);
  FFrameCreated := True;

  LabelBallCount.Text := 'Balls: 0';
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
    Inc(FBallCount);
  end;

  LabelBallCount.Text := 'Balls: ' + IntToStr(FBallCount);
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
begin
  TimerPhysics.Enabled := False;
  try
    //forward !
    FWorld.Step(1.0 / 60.0);

    //FPS
    Inc(FFrameCount);
    ElapsedMs := FStopwatch.ElapsedMilliseconds;
    if ElapsedMs - FLastFPSUpdate >= 500 then
    begin
      LabelFPS.Text := Format('FPS: %.1f', [FFrameCount * 1000 / (ElapsedMs - FLastFPSUpdate)]);
      FFrameCount := 0;
      FLastFPSUpdate := ElapsedMs;
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
  if Canvas.BeginScene then
  try
    Canvas.Clear(TAlphaColorRec.White);
    RenderWorld(Canvas);
  finally
    Canvas.EndScene;
  end;
end;

procedure TFormGSPhy.RenderWorld(Canvas: TCanvas);
const
  MAX_SPEED = 15.0;  // Max velocity.
var
  I: Integer;
  P: PPhyParticle;
  Box: PPhyBox;
  Rect: TRectF;
  W, H: Single;
  VelX, VelY, Speed, SpeedNorm: Single;
  ColorR, ColorG, ColorB: Byte;
begin
  W := ClientWidth;
  H := ClientHeight;

  Canvas.Fill.Color := TAlphaColorRec.Lightgray;
  Canvas.FillRect(RectF(0, 0, W, 2), 0, 0, [], 1.0);           // up
  Canvas.FillRect(RectF(0, H - 2, W, H), 0, 0, [], 1.0);       // down
  Canvas.FillRect(RectF(0, 0, 2, H), 0, 0, [], 1.0);           // left
  Canvas.FillRect(RectF(W - 2, 0, W, H), 0, 0, [], 1.0);       // right -> ;)

  // Box
  Canvas.Fill.Color := TAlphaColorRec.Gray;
  for I := 0 to FWorld.BoxCount - 1 do
  begin
    Box := FWorld.GetBox(I);
    Rect := RectF(Box^.MinX, Box^.MinY, Box^.MaxX, Box^.MaxY);
    Canvas.FillRect(Rect, 0, 0, [], 1.0);
  end;

  // Particles.
  for I := 0 to FWorld.ParticleCount - 1 do
  begin
    P := FWorld.GetParticle(I);

    // Velocity calculus (Verlet: V = Pos - OldPos)
    VelX := P^.Pos.X - P^.OldPos.X;
    VelY := P^.Pos.Y - P^.OldPos.Y;
    Speed := Sqrt(VelX * VelX + VelY * VelY);

    // Normaliser entre 0 et 1 (?)
    SpeedNorm := Speed / MAX_SPEED;
    if SpeedNorm > 1.0 then SpeedNorm := 1.0;

    // Speed vs color.
    if SpeedNorm < 0.25 then
    begin
      // Blue -> Cyan
      ColorR := 0;
      ColorG := Round(SpeedNorm * 4 * 255);
      ColorB := 255;
    end
    else if SpeedNorm < 0.5 then
    begin
      // Cyan -> Green
      ColorR := 0;
      ColorG := 255;
      ColorB := Round((1 - (SpeedNorm - 0.25) * 4) * 255);
    end
    else if SpeedNorm < 0.75 then
    begin
      // Green -> Yellow
      ColorR := Round((SpeedNorm - 0.5) * 4 * 255);
      ColorG := 255;
      ColorB := 0;
    end
    else
    begin
      // Yellow -> red
      ColorR := 255;
      ColorG := Round((1 - (SpeedNorm - 0.75) * 4) * 255);
      ColorB := 0;
    end;

    Canvas.Fill.Color := TAlphaColor($FF000000 or (ColorR shl 16) or (ColorG shl 8) or ColorB);

    Rect := RectF(
      P^.Pos.X - P^.Radius,
      P^.Pos.Y - P^.Radius,
      P^.Pos.X + P^.Radius,
      P^.Pos.Y + P^.Radius
    );
    Canvas.FillEllipse(Rect, 1.0);
    Canvas.Stroke.Color := TAlphaColorRec.Black;
    Canvas.Stroke.Thickness := 2;
    Canvas.DrawEllipse(Rect, 1.0);
  end;
end;

end.
