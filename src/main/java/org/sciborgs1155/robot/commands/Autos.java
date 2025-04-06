package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.advance;
import static org.sciborgs1155.robot.Constants.allianceReflect;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.*;
import java.util.function.Supplier;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.Fault;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.robot.FieldConstants.Branch;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.elevator.*;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import org.sciborgs1155.robot.scoral.Scoral;

public class Autos {
  @NotLogged
  public static SendableChooser<Command> configureAutos(
      Drive drive, Scoraling scoraling, Elevator elevator, Alignment alignment, Scoral scoral) {
    SendableChooser<Command> chooser = new SendableChooser<>();
    chooser.addOption("no auto", Commands.none());
    chooser.addOption("B4", B4(alignment, scoraling));
    chooser.addOption("P4", P4(alignment, scoraling));
    chooser.addOption("nyc practice field", nycPracticeField(alignment, scoraling));
    chooser.addOption("at home (bad middle reef)", badHome(alignment, scoraling));
    chooser.setDefaultOption(
        "line",
        drive.run(
            () ->
                drive.setChassisSpeeds(
                    new ChassisSpeeds(0.5, 0, 0),
                    ControlMode.OPEN_LOOP_VELOCITY,
                    elevator.position())));
    // chooser.addOption("practice field", test(alignment, scoraling, drive::resetOdometry));
    chooser.addOption(
        "wheel characteirzatiion",
        DriveCommands.wheelRadiusCharacterization(drive).withTimeout(Seconds.of(8)));
    return chooser;
  }

  /**
   * A smart auto which utilizes alignment for planning. It will dynamically replan based on whether
   * is has a coral or not; see alignSource() and alignReef().
   *
   * @param branches an <i>ordered</i> list representing what branches to score in auto. May be any
   *     length greater than or equal to 1.
   */
  public static Command alignAuto(Alignment alignment, Scoraling scoraling, List<Branch> branches) {
    if (branches.isEmpty()) {
      FaultLogger.report(
          new Fault("alignAuto fault", "alignAuto passed zero branches", FaultType.ERROR));
      return Commands.none();
    }

    return branches.stream()
        .map(
            b ->
                Commands.sequence(
                    alignSource(alignment, scoraling, 0).ignoringDisable(true),
                    alignReef(b, alignment, scoraling).ignoringDisable(true)))
        .reduce(Commands.none(), (a, b) -> a.andThen(b))
        .ignoringDisable(false);
  }

  public static Command fakeAlignAuto(
      Alignment alignment, Scoraling scoraling, List<Branch> branches) {
    if (branches.isEmpty()) {
      FaultLogger.report(
          new Fault("alignAuto fault", "alignAuto passed zero branches", FaultType.ERROR));
      return Commands.none();
    }

    return branches.stream()
        .map(
            b ->
                Commands.sequence(
                    scoraling.hpsIntake().onlyIf(() -> !scoraling.hasCoral()).asProxy(),
                    alignReef(b, alignment, scoraling),
                    alignment.moveRobotRelative(advance(Inches.of(-15))).asProxy()))
        .reduce(Commands.none(), (a, b) -> a.andThen(b));
  }

  /**
   * Aligns to the reef with appropriate timeouts. It will end if it does not possess a coral.
   *
   * @param branch the branch to score on.
   */
  public static Command alignReef(Branch branch, Alignment alignment, Scoraling scoraling) {
    return alignment
        .reef(Level.L4, branch)
        .ignoringDisable(true)
        .withTimeout(6)
        .onlyIf(scoraling::hasCoral)
        .ignoringDisable(false);
  }

  /**
   * Aligns to the source with appropriate timeouts. It will end if it already has a coral. It will
   * retry intaking a specified number of times.
   *
   * @param retries the number of times to retry (0 indicates to only run the command once)
   */
  public static Command alignSource(Alignment alignment, Scoraling scoraling, int retries) {
    Supplier<Command> attempt =
        () ->
            // (alignment
            //         .source()
            //         .until(alignment::stalling)
            //         .andThen(
            //             alignment
            //                 .moveRobotRelative(
            //                     rotate(Rotation2d.kCW_90deg).plus(strafe(Meters.of(0.2))))
            //                 .onlyIf(alignment::stalling)
            //                 .withTimeout(1)))
            //     .repeatedly()
            //     .until(alignment::atGoal)
            //     .andThen(scoraling.hpsIntake().withTimeout(2.5).asProxy())
            alignment
                .source()
                .ignoringDisable(true)
                .withDeadline(scoraling.hpsIntake().ignoringDisable(true).asProxy())
                .withTimeout(5);

    Command source = attempt.get();

    for (int i = 0; i < retries; i++) {
      source =
          Commands.sequence(
              source,
              Commands.parallel(
                  alignment
                      .moveRobotRelative(advance(Meters.of(0.2)))
                      .ignoringDisable(true)
                      .asProxy(),
                  scoraling.retryIntake().ignoringDisable(true)),
              attempt.get());
    }

    return Commands.race(source, Commands.waitUntil(scoraling::hasCoral).ignoringDisable(true))
        .ignoringDisable(false);
  }

  /** Runas a "bottom" side auto with 4 L4 coral scored. */
  public static Command B4(Alignment alignment, Scoraling scoraling) {
    return alignAuto(alignment, scoraling, List.of(Branch.I, Branch.K, Branch.L, Branch.J));
  }

  /** runs a proceser side auto with 4 L4 coral scored. */
  public static Command P4(Alignment alignment, Scoraling scoraling) {
    return alignAuto(alignment, scoraling, List.of(Branch.E, Branch.D, Branch.C, Branch.B));
  }

  public static Command nycPracticeField(Alignment alignment, Scoraling scoraling) {
    return alignAuto(alignment, scoraling, List.of(Branch.L, Branch.A, Branch.K));
  }

  public static Command badHome(Alignment alignment, Scoraling scoraling) {
    return fakeAlignAuto(alignment, scoraling, List.of(Branch.I, Branch.J));
    // return alignAuto(alignment, scoraling, List.of(Branch.A, Branch.B));
  }

  // * Warms up the pathfind command by telling drive to drive to itself. */
  public static Command warmupCommand(Drive drive, Alignment alignment, Scoraling scoraling) {
    // return Commands.none();
    return Commands.sequence(
            drive
                .runOnce(() -> drive.resetOdometry(allianceReflect(Pose2d.kZero)))
                .ignoringDisable(true),
            P4(alignment, scoraling).ignoringDisable(true).withTimeout(0.04),
            B4(alignment, scoraling).ignoringDisable(true).withTimeout(0.04),
            Commands.runOnce(() -> FaultLogger.report("autos", "Finished warmup", FaultType.INFO))
                .ignoringDisable(true))
        .ignoringDisable(true);
  }
}
