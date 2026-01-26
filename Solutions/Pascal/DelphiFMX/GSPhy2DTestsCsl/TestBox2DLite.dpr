program TestBox2DLite;

{$APPTYPE CONSOLE}

uses
  System.SysUtils,
  GS.Phy.Box2DLite;

var
  World: TB2LWorld;
  Ground, Box1: TB2LBody;
  Step: Integer;
  Contacts: array[0..1] of TB2LContact;
  NumContacts: Integer;
begin
  WriteLn('=== Test Box2DLite ===');
  WriteLn;

  // Create world with gravity (0, -10)
  World := TB2LWorld.Create(TB2LVec2.Create(0, -10), 10);
  try
    // Ground: wide, thin, at Y=0.5 (top edge at Y=1)
    Ground := World.AddBody(TB2LVec2.Create(100, 1), B2L_MAX_FLOAT);  // Static
    Ground.Position := TB2LVec2.Create(0, 0.5);
    Ground.Friction := 0.3;
    WriteLn(Format('Ground: Pos(%.1f, %.1f) Size(%.1f, %.1f) InvMass=%.3f',
      [Ground.Position.X, Ground.Position.Y, Ground.Width.X, Ground.Width.Y, Ground.InvMass]));

    // Box: 2x2, starts at Y=2.5 (just above ground), should hit ground quickly
    Box1 := World.AddBody(TB2LVec2.Create(2, 2), 10);  // Dynamic, mass=10
    Box1.Position := TB2LVec2.Create(0, 2.5);  // Close to ground (ground top is at Y=1)
    Box1.Rotation := 0.1;  // Slight angle
    Box1.Friction := 0.3;
    WriteLn(Format('Box1:   Pos(%.1f, %.1f) Size(%.1f, %.1f) InvMass=%.3f',
      [Box1.Position.X, Box1.Position.Y, Box1.Width.X, Box1.Width.Y, Box1.InvMass]));

    WriteLn;
    WriteLn('Simulating 300 steps (5 seconds at 60Hz)...');
    WriteLn;

    for Step := 1 to 300 do
    begin
      World.Step(1/60);

      // Test collision directly AFTER step
      NumContacts := B2LCollide(Contacts, Ground, Box1);

      // Print every 10 steps or first 5 or when collision detected
      if (Step mod 10 = 0) or (Step <= 5) or (NumContacts > 0) then
      begin
        WriteLn(Format('Step %3d: Box.Y=%.3f Vel.Y=%.3f Contacts=%d Arbiters=%d',
          [Step, Box1.Position.Y, Box1.Velocity.Y, NumContacts, World.ArbiterCount]));
      end;

      // Stop if box went way below ground (collision failed)
      if Box1.Position.Y < -10 then
      begin
        WriteLn('*** FAIL: BOX FELL THROUGH GROUND! ***');
        Break;
      end;

      // Stop if box stabilized on ground
      if (Box1.Position.Y < 2.5) and (Abs(Box1.Velocity.Y) < 0.1) and (World.ArbiterCount > 0) then
      begin
        WriteLn('*** SUCCESS: BOX STABILIZED ON GROUND ***');
        Break;
      end;
    end;

    WriteLn;
    WriteLn(Format('Final: Pos(%.3f, %.3f) Vel(%.3f, %.3f) Rot=%.3f',
      [Box1.Position.X, Box1.Position.Y, Box1.Velocity.X, Box1.Velocity.Y, Box1.Rotation]));
    WriteLn(Format('Arbiters: %d', [World.ArbiterCount]));

  finally
    World.Free;
  end;

  WriteLn;
  WriteLn('Test complete.');
end.
