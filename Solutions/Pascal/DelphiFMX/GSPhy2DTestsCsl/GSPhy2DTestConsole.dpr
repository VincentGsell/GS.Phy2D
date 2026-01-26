program GSPhy2DTestConsole;

{$APPTYPE CONSOLE}

{$R *.res}

uses
  System.SysUtils,
  GS.Phy.AABB in '..\..\src\GS.Phy.AABB.pas',
  GS.Phy.Renderer.FMX in '..\..\src\GS.Phy.Renderer.FMX.pas',
  GS.Phy.Renderer in '..\..\src\GS.Phy.Renderer.pas',
  GS.Phy.SpatialHash in '..\..\src\GS.Phy.SpatialHash.pas',
  GS.Phy.Types in '..\..\src\GS.Phy.Types.pas',
  GS.Phy.Vec2 in '..\..\src\GS.Phy.Vec2.pas',
  GS.Phy.World in '..\..\src\GS.Phy.World.pas';

begin
  try
    { TODO -oUser -cConsole Main : Insert code here }
  except
    on E: Exception do
      Writeln(E.ClassName, ': ', E.Message);
  end;
end.
