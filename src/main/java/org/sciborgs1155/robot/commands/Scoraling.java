package org.sciborgs1155.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
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

    /*
    Causes the intaking command to end if the coral reaches the desired state between the hps and scoral
    beambreaks.
    */
    hopper
        .beambreakTrigger
        .negate()
        .or(scoral.beambreakTrigger)
        .onFalse(stop().onlyIf(() -> hopper.getCurrentCommand().getName().equals("intakingHPS")));
  }

  /** A command which intakes from the human player station. */
  public Command hpsIntake() {
    return elevator
        .retract()
        .alongWith(Commands.waitUntil(elevator::atGoal).andThen(runRollersForw()))
        .onlyIf(scoral.beambreakTrigger)
        .withName("intakingHPS");
  }

  /**
   * A command which scores a coral at the given level, assuming you are already at the correct
   * branch.
   *
   * @param level the level the scoral scores in
   */
  public Command scoral(Level level) {
    return elevator
        .scoreLevel(level)
        .alongWith(Commands.waitUntil(elevator::atGoal).andThen(scoral.score()))
        .withName("scoraling");
  }

  /**
   * A command which grabs the algae from above the level given; only L2 and L3 are allowed.
   *
   * @param level the level to score above
   */
  public Command cleanAlgae(Level level) {
    return elevator
        .clean(level)
        .alongWith(Commands.waitUntil(elevator::atGoal).andThen(scoral.score()))
        .onlyIf(scoral.beambreakTrigger)
        .withName("cleanAlgae");
  }

  /** A command which halts both the hopper and the scoral. */
  public Command stop() {
    return hopper.stop().alongWith(scoral.stop()).withName("stopping");
  }

  /** A command which runs the hps + scoral rollers forward (generally as a form of intaking). */
  public Command runRollersForw() {
    return hopper.intake().alongWith(scoral.score()).withName("runningRollers");
  }

    /** A command which runs the hps + scoral rollers forward (generally as a form of intaking). */
  public Command runRollersBack() {
      return hopper.outtake().alongWith(scoral.algae()).withName("runningRollers");
  }
}
