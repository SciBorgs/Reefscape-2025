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

    hopper
        .beambreakTrigger
        .negate()
        .or(scoral.beambreakTrigger)
        .onFalse(stop().onlyIf(() -> hopper.getCurrentCommand().getName().equals("IntakingHPS")));
  }

  /** intakes from the human player station */
  public Command hpsIntake() {
    return elevator
        .retract()
        .andThen(runRollers())
        .onlyIf(scoral.beambreakTrigger)
        .withName("IntakingHPS");
  }

  /**
   * scores a coral at the given level, assuming you are already at the correct branch
   *
   * @param level the level the scoral scores in
   */
  public Command scoral(Level level) {
    return elevator.scoreLevel(level).andThen(scoral.outtake()).withName("scoraling");
  }

  /** grabs the algae from the level given (goes above the level) ONLY L2 and L3 */
  public Command grabAlgae(Level level) {
    return elevator
        .clean(level)
        .andThen(scoral.intake())
        .onlyIf(scoral.beambreakTrigger)
        .withName("cleanAlgae");
  }

  /** halts both the hopper and the scoral */
  public Command stop() {
    return hopper.stop().alongWith(scoral.stop()).withName("stopping");
  }

  /** runs the hps + scoral rollers forward (intaking) */
  public Command runRollers() {
    return hopper.intake().alongWith(scoral.outtake()).withName("runningRollers");
  }
}
