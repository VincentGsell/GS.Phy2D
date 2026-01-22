program GSPhy2DTestConsole;

{$APPTYPE CONSOLE}

{$R *.res}

uses
  System.SysUtils,
  GS.Phy.World in '..\..\..\..\Sources\GS.Phy.World.pas',
  GS.Phy.Vec2 in '..\..\..\..\Sources\GS.Phy.Vec2.pas',
  GS.Phy.Types in '..\..\..\..\Sources\GS.Phy.Types.pas',
  GS.Phy.SpatialHash in '..\..\..\..\Sources\GS.Phy.SpatialHash.pas',
  GS.Phy.Renderer in '..\..\..\..\Sources\GS.Phy.Renderer.pas',
  GS.Phy.Renderer.FMX in '..\..\..\..\Sources\GS.Phy.Renderer.FMX.pas',
  GS.Phy.AABB in '..\..\..\..\Sources\GS.Phy.AABB.pas';

begin
  try
    { TODO -oUser -cConsole Main : Insert code here }
  except
    on E: Exception do
      Writeln(E.ClassName, ': ', E.Message);
  end;
end.
