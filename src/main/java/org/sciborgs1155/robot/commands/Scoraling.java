package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.lib.Assertion.tAssert;
import static org.sciborgs1155.robot.drive.DriveConstants.driveSim;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.CORAL_FROM_CARRIAGE;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.FieldConstants.Source;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.led.LEDs;
import org.sciborgs1155.robot.scoral.Scoral;

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

    // scoral.blocked.onTrue(Commands.runOnce(() -> stop = true));
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
                .alongWith(
                    Commands.waitUntil(elevator::atGoal)
                        .andThen(
                            runRollers().alongWith(Commands.defer(() -> dropCoral(), Set.of()))))
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
   * A command which scores at the given level, assuming that you are already at the correct branch
   * and moving the elevator toward the correct position.
   *
   * @param level the level the scoral will score in
   */
  public Command scoralWithoutElevator(Level level) {
    return Commands.waitUntil(elevator::atGoal)
        .withTimeout(1.5)
        .andThen(scoral.score(level).asProxy().alongWith(shootCoral()))
        .withName("scoraling without elevator");
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
        .andThen(leds.blink(Color.kGold).onlyIf(() -> !scoral.blocked.getAsBoolean()))
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

  /**
   * @return a boolean designating whether a coral indeed maintains a presence within the scoral's
   *     mechanism
   */
  public Trigger hasCoral() {
    return scoral.blocked;
  }

  /**
   * Drops the coral from the closest human player station into the robot. It is used for simulation
   * purposes along with maplesim
   */
  public Command dropCoral() {
    if (Robot.isReal()) {
      return Commands.none();
    }

    return Commands.runOnce(
        () ->
            SimulatedArena.getInstance()
                .addGamePieceProjectile(
                    new ReefscapeCoralOnFly(
                        Source.nearest(Robot.maplePose())
                            .getTranslation()
                            .plus(new Translation2d(.4, -.4)),
                        new Translation2d(),
                        new ChassisSpeeds(),
                        new Rotation2d(-Math.PI / 4),
                        Meters.of(1.2),
                        MetersPerSecond.of(-3),
                        Radians.of(Math.PI / 4))));
  }

  /**
   * Shoots the coral from the scoral position onto the branches. It is used for simulation purposes
   * in maplesim
   */
  public Command shootCoral() {
    if (Robot.isReal()) {
      return Commands.none();
    }

    return Commands.runOnce(
        () ->
            SimulatedArena.getInstance()
                .addGamePieceProjectile(
                    new ReefscapeCoralOnFly(
                        new Translation2d(heldCoral().getX(), heldCoral().getY()),
                        new Translation2d(0.2, 0),
                        driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        new Rotation2d(heldCoral().getRotation().getMeasureZ()),
                        heldCoral().getMeasureZ(),
                        MetersPerSecond.of(3),
                        Radians.of(-Math.PI / 2))));
  }

  /***
   * @return the pose of a coral if held in the scoral
   * Used to visualize coral in simulation.
   */
  @Logged
  public Pose3d heldCoral() {
    if (!scoral.blocked.getAsBoolean()) {
      return new Pose3d();
    }

    return new Pose3d(Robot.maplePose())
        .plus(elevator.carriage().minus(new Pose3d()))
        .plus(CORAL_FROM_CARRIAGE);
  }
}
