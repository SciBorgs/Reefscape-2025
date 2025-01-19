package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  public static Arm create() {
    return new Arm();
  }

  public Command goTo(Angle angle) {
    return run(
        () ->
            System.out.println(
                "My name is Mr Arm and im going to: " + angle.in(Degrees) + "degrees!"));
  }
}
