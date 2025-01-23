package org.sciborgs1155.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.sciborgs1155.robot.Constants.Field.Level;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.scoral.Scoral;

public class Scoraling {
  private final Hopper hopper;
  private final Scoral scoral;
  private final Elevator elevator;

  public Scoraling(Hopper hopper, Scoral scoral, Elevator elevator) {
    this.hopper = hopper;
    this.scoral = scoral;
    this.elevator = elevator;

    hopper.beambreakTrigger.onTrue(hopper.stop().alongWith(scoral.stop()));
    scoral.beambreakTrigger.onFalse(hopper.stop().alongWith(scoral.stop()));
  }

  public Command hpsIntake() {
    return elevator.retract().andThen(run()).onlyIf(scoral.beambreakTrigger);
  }

  public Command scoral(Level level) {
    return elevator.scoreLevel(level).andThen(scoral.outtake());
  }

  /** ONLY L2 and L3 */
  public Command grabAlgae(Level level) {
    return elevator.clean(level).andThen(scoral.intake()).onlyIf(scoral.beambreakTrigger);
  }

  public Command halt() {
    return hopper.stop().alongWith(scoral.stop());
  }

  public Command run() {
    return hopper.intake().alongWith(scoral.outtake());
  }
}
