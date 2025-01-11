package org.sciborgs1155.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.sciborgs1155.robot.arm.Arm;
import org.sciborgs1155.robot.arm.ArmConstants;
import org.sciborgs1155.robot.roller.Roller;

public class GroundIntake {

  private Arm arm;
  private Roller roller;

  public GroundIntake(Arm arm, Roller roller) {
    this.arm = arm;
    this.roller = roller;
  }

  public Command intake() {
    return arm.goTo(ArmConstants.INTAKE_ANGLE).alongWith(roller.intake());
  }

  public Command outtake() {
    return arm.goTo(ArmConstants.OUTTAKE_ANGLE).andThen(roller.outtake());
  }

  public Command climb() {
    return arm.goTo(ArmConstants.CLIMB_INTAKE_ANGLE);
  }
}
