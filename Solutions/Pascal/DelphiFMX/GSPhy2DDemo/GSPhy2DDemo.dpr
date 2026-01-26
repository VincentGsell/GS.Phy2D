program GSPhy2DDemo;

uses
  System.StartUpCopy,
  FMX.Forms,
  fmain in 'fmain.pas' {FormGSPhy},
  GS.Phy.AABB in '..\..\src\GS.Phy.AABB.pas',
  GS.Phy.Renderer.FMX in '..\..\src\GS.Phy.Renderer.FMX.pas',
  GS.Phy.Renderer in '..\..\src\GS.Phy.Renderer.pas',
  GS.Phy.SpatialHash in '..\..\src\GS.Phy.SpatialHash.pas',
  GS.Phy.Types in '..\..\src\GS.Phy.Types.pas',
  GS.Phy.Vec2 in '..\..\src\GS.Phy.Vec2.pas',
  GS.Phy.World in '..\..\src\GS.Phy.World.pas';

{$R *.res}

begin
  Application.Initialize;
  Application.CreateForm(TFormGSPhy, FormGSPhy);
  Application.Run;
end.
