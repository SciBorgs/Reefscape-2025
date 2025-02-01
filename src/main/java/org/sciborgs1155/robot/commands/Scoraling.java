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
  }

  public Command hpsIntake() {
    // elevator goes to default position with retract
    // andThen -> hopper and scoral outtakes (since outtakaing will intake this way) until the
    // scoral beambreak is false OR the hps beambreak is true
    // onlyIf -> scoral beambreak is true (don't have a coral)

    return Commands.idle(hopper, scoral, elevator);
  }

  public Command scoral(Level level) {
    // elevator goes to the level selected
    // andThen -> scoral outtakes until its beambreak is true

    return Commands.idle(scoral, elevator);
  }

  public Command grabAlgae(double algaeLevel) {
    // will make a constant or something for the algae heights needed

    // elevator goes to the algae height
    // andThen -> intake with scoral
    // onlyIf -> scoral beambreak is true (don't have a coral)

    return Commands.idle(scoral, elevator);
  }
}
