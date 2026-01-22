unit fmain;

interface

uses
  System.SysUtils, System.Types, System.UITypes, System.Classes,
  System.Diagnostics, FMX.Types, FMX.Controls, FMX.Forms, FMX.Graphics,
  FMX.Dialogs, FMX.SpinBox, FMX.Controls.Presentation, FMX.StdCtrls, FMX.Layouts,
  GS.Phy.Vec2, GS.Phy.Types, GS.Phy.World, GS.Phy.Renderer, GS.Phy.Renderer.FMX,
  FMX.Edit, FMX.EditBox, FMX.ActnList;

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
    procedure FormMouseDown(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);
    procedure FormMouseMove(Sender: TObject; Shift: TShiftState; X, Y: Single);
    procedure FormMouseUp(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);
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
    FBallCount: Integer;
    FMouseBox: Integer;
    procedure CreateFrame;
    procedure UpdateFrame;
    procedure SpawnBalls(Count: Integer);
    procedure SpawnBoxes(Count: Integer);
    procedure RenderWorld;
  end;

var
  FormGSPhy: TFormGSPhy;

implementation

uses
  System.Threading;

{$R *.fmx}

const
  WALL_THICKNESS = 20;
  BALL_RADIUS_MIN = 2;
  BALL_RADIUS_MAX = 2;

procedure TFormGSPhy.FormCreate(Sender: TObject);
begin
  FFrameCreated := False;
  FBallCount := 0;

  FWorld := TPhyWorld.Create;
  FWorld.SetGravity(0, 800);
  FWorld.Damping := 0.99;
  FWorld.CollisionIterations := 2;

  FRenderer := TPhyRendererFMX.Create;

  SpinBoxIterations.Value := FWorld.CollisionIterations;

  FStopwatch := TStopwatch.StartNew;
  FFrameCount := 0;
  FLastFPSUpdate := 0;

  TTask.Run(
    procedure
    begin
      while TTask.CurrentTask.Status in [TTaskStatus.Running] do
      begin
        //TMonitor.Enter(FWorld);
        try
          FWorld.Step(1.0 / 60.0);
        finally
          //TMonitor.Exit(FWorld);
        end;
        Sleep(10);
      end;
    end);
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

  TMonitor.Enter(FWorld);
  try
    FWorld.Clear;
    FBallCount := 0;

    FWorld.SetWorldBounds(W, H);
    FFrameCreated := True;

    LabelBallCount.Text := 'Balls: 0';
    FMouseBox := FWorld.AddBox(200, 200, 40, 40);
  finally
    TMonitor.Exit(FWorld);
  end;
end;

procedure TFormGSPhy.UpdateFrame;
var
  W, H: Single;
begin
  W := ClientWidth;
  H := ClientHeight;
  TMonitor.Enter(FWorld);
  try
    FWorld.SetWorldBounds(W, H);
  finally
    TMonitor.Exit(FWorld);
  end;
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
    //FWorld.Step(1.0 / 60.0);

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

procedure TFormGSPhy.FormMouseDown(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);
begin
  var Box := FWorld.GetBox(FMouseBox);
  Box^.MinX := X - 40 * 0.5;
  Box^.MaxX := X + 40 * 0.5;
  Box^.MinY := Y - 40 * 0.5;
  Box^.MaxY := Y + 40 * 0.5;
  FWorld.Boxes[FMouseBox] := Box^;
  FWorld.NeedRebuildBoxGrid;
end;

procedure TFormGSPhy.FormMouseMove(Sender: TObject; Shift: TShiftState; X, Y: Single);
begin
  if ssLeft in Shift then
  begin
    var Box := FWorld.GetBox(FMouseBox);
    Box^.MinX := X - 40 * 0.5;
    Box^.MaxX := X + 40 * 0.5;
    Box^.MinY := Y - 40 * 0.5;
    Box^.MaxY := Y + 40 * 0.5;
    FWorld.Boxes[FMouseBox] := Box^;
    FWorld.NeedRebuildBoxGrid;
  end;
end;

procedure TFormGSPhy.FormMouseUp(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);
begin
  var Box := FWorld.GetBox(FMouseBox);
  Box^.MinX := -40 - 40 * 0.5;
  Box^.MaxX := -40 + 40 * 0.5;
  Box^.MinY := -40 - 40 * 0.5;
  Box^.MaxY := -40 + 40 * 0.5;
  FWorld.Boxes[FMouseBox] := Box^;
  FWorld.NeedRebuildBoxGrid;
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

procedure TFormGSPhy.FormPaint(Sender: TObject; Canvas: TCanvas; const ARect: TRectF);
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
  VelX, VelY: Single;
  ParticleColor: TPhyColor;
  W, H: Single;
begin
  W := FRenderer.Width;
  H := FRenderer.Height;

  // Bordures du monde
  FRenderer.FillRect(0, 0, W, 2, TPhyColors.LightGray);
  FRenderer.FillRect(0, H - 2, W, H, TPhyColors.LightGray);
  FRenderer.FillRect(0, 0, 2, H, TPhyColors.LightGray);
  FRenderer.FillRect(W - 2, 0, W, H, TPhyColors.LightGray);

  // Boxes statiques
  //TMonitor.Enter(FWorld);
  try
    for I := 0 to FWorld.BoxCount - 1 do
    begin
      Box := FWorld.GetBox(I);
      FRenderer.FillRect(Box^.MinX, Box^.MinY, Box^.MaxX, Box^.MaxY, TPhyColors.Gray);
    end;

    // Particules avec couleur selon velocite
    for I := 0 to FWorld.ParticleCount - 1 do
    begin
      PosX := FWorld.GetPosX(I);
      PosY := FWorld.GetPosY(I);
      OldPosX := FWorld.GetOldPosX(I);
      OldPosY := FWorld.GetOldPosY(I);
      Radius := FWorld.GetRadius(I);

      // Velocite Verlet: V = Pos - OldPos
      VelX := PosX - OldPosX;
      VelY := PosY - OldPosY;
                                        // TAlphaColors.Darkseagreen;//
      ParticleColor :=   FRenderer.VelocityToColor(VelX, VelY);

      FRenderer.FillCircle(PosX, PosY, Radius * 3, ParticleColor);
      //FRenderer.DrawCircle(PosX, PosY, Radius, TPhyColors.Black, 2.0);
    end;
  finally
    //TMonitor.Exit(FWorld);
  end;
end;

end.

