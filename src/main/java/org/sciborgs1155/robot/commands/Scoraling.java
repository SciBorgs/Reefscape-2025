package org.sciborgs1155.robot.commands;

import java.util.Set;

import org.sciborgs1155.lib.Assertion;
import static org.sciborgs1155.lib.Assertion.tAssert;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.led.LEDs;
import org.sciborgs1155.robot.scoral.Scoral;

import edu.wpi.first.epilogue.Logged;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Scoraling {
  private final Hopper hopper;
  private final Scoral scoral;
  private final Elevator elevator;
  private final LEDs leds;

  public Scoraling(Hopper hopper, Scoral scoral, Elevator elevator, LEDs leds) {
    this.hopper = hopper;
    this.scoral = scoral;
    this.elevator = elevator;
    this.leds = leds;

    /*
    Causes the intaking command to end if the coral reaches the desired state between the hps and scoral
    beambreaks.
    */
    hopper.blocked.negate().or(scoral.blocked).onTrue(Commands.runOnce(() -> stop = true));
  }

  @Logged private boolean stop = false;

  public Command noElevatorIntake() {
    return Commands.runOnce(() -> stop = false)
        .andThen(runRollers().repeatedly())
        .until(() -> stop)
        .finallyDo(() -> stop = false)
        .withName("no elevator intake");
  }

  /** A command which intakes from the human player station. */
  public Command hpsIntake() {
    return Commands.runOnce(() -> stop = false)
        .andThen(
            elevator
                .retract()
                .alongWith(Commands.waitUntil(elevator::atGoal).andThen(runRollers()))
                .until(() -> stop)
                .finallyDo(() -> stop = false))
        .withName("intakingHPS");
  }

  /** A command that retracts the elevator. */
  public Command retract() {
    return elevator.retract().withName("retractElevator");
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
        .alongWith(
            Commands.waitUntil(elevator::atGoal).andThen(scoral.score()),
            leds.progressGradient(
                () -> 1 - elevator.position() / level.extension.in(Meters), elevator::atGoal))
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
        .alongWith(
            Commands.waitUntil(elevator::atGoal).andThen(scoral.score()),
            leds.progressGradient(
                () -> 1 - elevator.position() / level.extension.in(Meters), elevator::atGoal))
        .onlyIf(scoral.blocked.negate())
        .withName("cleanAlgae");
  }

  /** A command which halts both the hopper and the scoral. */
  public Command stop() {
    return hopper.stop().alongWith(scoral.stop()).withName("stopping");
  }

  public Command retryIntake() {
    return elevator
        .scoreLevel(Level.L1)
        .withTimeout(Seconds.of(0.2))
        .asProxy()
        .alongWith(runRollersBack().asProxy().until(hopper.blocked.negate()));
  }

  /**
   * A command which runs the hps + scoral rollers forward (generally as a form of intaking), then
   * runs them back and forth until a coral enters the scoral.
   */
  public Command runRollers() {
    return hopper
        .intake()
        .alongWith(scoral.intake())
        .andThen(leds.blink(Color.kGold).onlyIf(() -> !scoral.beambreak.get()))
        // .andThen((runRollersBack().withTimeout(0.2).onlyIf(hopper.beambreakTrigger.negate())))
        .withName("runningRollers");
  }

  /** A command which runs the hps + scoral rollers forward (generally as a form of intaking). */
  public Command runRollersBack() {
    return hopper.outtake().alongWith(scoral.algae()).withName("runningRollers");
  }

  public Test runRollersTest() {
    Command testCommand =
        Commands.runOnce(() -> stop = false)
            .andThen(runRollers().asProxy())
            .until(() -> stop)
            .withTimeout(5)
            .finallyDo(() -> stop = false);
    Assertion hasCoral =
        tAssert(
            scoral.blocked, "scoral beambreak blocked", () -> "" + scoral.blocked.getAsBoolean());
    return new Test(testCommand, Set.of(hasCoral));
  }

  public boolean hasCoral() {
    return scoral.blocked.getAsBoolean();
  }
}
