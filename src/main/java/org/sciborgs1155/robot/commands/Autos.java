package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.advance;

import edu.wpi.first.epilogue.NotLogged;
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
                    alignSource(alignment, scoraling, 0), alignReef(b, alignment, scoraling)))
        .reduce(Commands.none(), (a, b) -> a.andThen(b));
  }

  /**
   * Aligns to the reef with appropriate timeouts. It will end if it does not possess a coral.
   *
   * @param branch the branch to score on.
   */
  public static Command alignReef(Branch branch, Alignment alignment, Scoraling scoraling) {
    return alignment.reef(Level.L4, branch).withTimeout(7).onlyIf(scoraling::hasCoral);
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
            alignment
                .source()
                .andThen(scoraling.hpsIntake().withTimeout(2.5).asProxy())
                .withTimeout(5);

    Command source = attempt.get();

    for (int i = 0; i < retries; i++) {
      source =
          Commands.sequence(
              source,
              Commands.parallel(
                  alignment.moveRobotRelative(advance(Meters.of(0.2))).asProxy(),
                  scoraling.retryIntake()),
              attempt.get());
    }

    return Commands.race(source, Commands.waitUntil(scoraling::hasCoral))
        .onlyIf(() -> !scoraling.hasCoral());
  }

  /** Runas a "bottom" side auto with 4 L4 coral scored. */
  public static Command B4(Alignment alignment, Scoraling scoraling) {
    return alignAuto(alignment, scoraling, List.of(Branch.I, Branch.K, Branch.L, Branch.J));
  }

  /** runs a proceser side auto with 4 L4 coral scored. */
  public static Command P4(Alignment alignment, Scoraling scoraling) {
    return alignAuto(alignment, scoraling, List.of(Branch.E, Branch.D, Branch.C, Branch.B));
  }

  public static Command badHome(Alignment alignment, Scoraling scoraling) {
    return alignAuto(alignment, scoraling, List.of(Branch.A, Branch.B));
  }
}
