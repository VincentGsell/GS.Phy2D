program GSPhy2DDemo;

uses
  System.StartUpCopy,
  FMX.Forms,
  fmain in 'fmain.pas' {FormGSPhy},
  GS.Phy.Vec2 in '..\..\..\Sources\GS.Phy.Vec2.pas',
  GS.Phy.Types in '..\..\..\Sources\GS.Phy.Types.pas',
  GS.Phy.SpatialHash in '..\..\..\Sources\GS.Phy.SpatialHash.pas',
  GS.Phy.World in '..\..\..\Sources\GS.Phy.World.pas',
  GS.Phy.Renderer.FMX in '..\..\..\Sources\GS.Phy.Renderer.FMX.pas',
  GS.Phy.Renderer in '..\..\..\Sources\GS.Phy.Renderer.pas';

{$R *.res}

begin
  Application.Initialize;
  Application.CreateForm(TFormGSPhy, FormGSPhy);
  Application.Run;
end.
