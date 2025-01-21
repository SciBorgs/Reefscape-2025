package org.sciborgs1155.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.robot.arm.Arm;
import org.sciborgs1155.robot.coroller.Coroller;

public class Corolling {
  private Arm arm;
  private Coroller coroller;

  public Corolling(Arm arm, Coroller coroller) {
    this.arm = arm;
    this.coroller = coroller;
  }

  /**
   * Moves the arm to the intake angle and then begins to intake. Can be used for both coral and
   * algae.
   *
   * @return A command for ground intaking.
   */
  public Command intake() {
    return Commands.idle(arm, coroller);
  }

  /**
   * Moves the arm to the processor angle and then outtakes.
   *
   * @return A command for outtaking algae in the processor.
   */
  public Command processor() {
    return Commands.idle(arm, coroller);
  }

  /**
   * Moves the arm to the trough angle and then outtakes.
   *
   * @return A command for outtaking coral into the trough (L1).
   */
  public Command trough() {
    return Commands.idle(arm, coroller);
  }
}
