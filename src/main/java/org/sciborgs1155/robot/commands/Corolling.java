package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  public Command coralIntake() {
    return arm.goTo(CORAL_INTAKE_ANGLE)
        .asProxy()
        .withDeadline(
            Commands.waitUntil(() -> arm.atPosition(CORAL_INTAKE_ANGLE.in(Radians)))
                .andThen(roller.algaeIntake().asProxy().withTimeout(1)))
        .withName("ground intake");
  }

  public Command algaeIntake() {
    return arm.goTo(ALGAE_INTAKE_ANGLE)
        .asProxy()
        .withDeadline(
            Commands.waitUntil(() -> arm.atPosition(ALGAE_INTAKE_ANGLE.in(Radians)))
                .andThen(roller.algaeIntake().asProxy().withTimeout(1)))
        .withName("ground intake");
  }
  /**
   * Moves the arm to the processor angle and then outtakes.
   *
   * @return A command for outtaking algae in the processor.
   */
  public Command processor() {
    return arm.goTo(PROCESSOR_OUTTAKE_ANGLE)
        .withDeadline(
            Commands.waitUntil(() -> arm.atPosition(PROCESSOR_OUTTAKE_ANGLE.in(Radians)))
                .andThen(roller.coralIntake().asProxy().withTimeout(1)))
        .withName("processor");
  }

  /**
   * Moves the arm to the trough angle and then outtakes.
   *
   * @return A command for outtaking coral into the trough (L1).
   */
  public Command trough() {
    return arm.goTo(TROUGH_OUTTAKE_ANGLE)
        .withDeadline(
            Commands.waitUntil(() -> arm.atPosition(TROUGH_OUTTAKE_ANGLE.in(Radians)))
                .andThen(roller.coralIntake().asProxy().withTimeout(1)))
        .withName("trough");
  }
}
