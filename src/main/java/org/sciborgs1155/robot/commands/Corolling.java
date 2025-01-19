package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import org.sciborgs1155.robot.arm.Arm;
import org.sciborgs1155.robot.coroller.Coroller;

public class Corolling {

  private Arm arm;
  private Coroller roller;

  public Corolling(Arm arm, Coroller roller) {
    this.arm = arm;
    this.roller = roller;
  }

  /**
   * Moves the arm to the intake angle and then begins to intake.
   *
   * @return A command for ground intaking.
   */
  public Command intake() {
    return arm.goTo(INTAKE_ANGLE).alongWith(roller.intake());
  }

  /**
   * Moves the arm to the processor angle and then outtakes.
   *
   * @return A command for outtaking algae in the processor.
   */
  public Command processor() {
    return arm.goTo(PROCESSOR_OUTTAKE_ANGLE).until(arm::atGoal).andThen(roller.outtake());
  }

  /**
   * Moves the arm to the trough angle and then outtakes.
   *
   * @return A command for outtaking coral into the trough (L1).
   */
  public Command trough() {
    return arm.goTo(TROUGH_OUTTAKE_ANGLE)
        .until(arm::atGoal)
        .andThen(roller.outtake())
        .alongWith(arm.goTo(TROUGH_OUTTAKE_ANGLE));
  }
}
