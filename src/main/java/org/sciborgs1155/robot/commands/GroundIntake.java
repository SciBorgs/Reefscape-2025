package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import org.sciborgs1155.robot.arm.Arm;
import org.sciborgs1155.robot.coroller.Coroller;

public class GroundIntake {

  private Arm arm;
  private Coroller roller;

  public GroundIntake(Arm arm, Coroller roller) {
    this.arm = arm;
    this.roller = roller;
  }

  public Command intake() {
    return arm.goTo(INTAKE_ANGLE).until(arm::atGoal).alongWith(roller.intake());
  }

  public Command outtake() {
    return arm.goTo(OUTTAKE_ANGLE).until(arm::atGoal).andThen(roller.outtake());
  }

  public Command climbSetup() {
    return arm.goTo(CLIMB_INTAKE_ANGLE);
  }

  public Command climbExecute() {
    return arm.run(() -> arm.currentLimit(CLIMB_LIMIT.in(Amps))).andThen(arm.goTo(CLIMB_FINAL_ANGLE));
  }
}
