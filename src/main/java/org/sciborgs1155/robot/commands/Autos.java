package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static org.sciborgs1155.robot.Constants.Robot.*;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_SPEED;
import static org.sciborgs1155.robot.drive.DriveConstants.MODULE_OFFSET;
import static org.sciborgs1155.robot.drive.DriveConstants.WHEEL_COF;
import static org.sciborgs1155.robot.drive.DriveConstants.WHEEL_RADIUS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.*;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.Fault;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.robot.Constants.Field.Branch;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.elevator.*;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;

public class Autos {
  public static SendableChooser<Command> configureAutos(
      Drive drive, Scoraling scoraling, Elevator elevator, Alignment alignment) {
    AutoBuilder.configure(
        drive::pose,
        drive::resetOdometry,
        drive::robotRelativeChassisSpeeds,
        (s, g) -> drive.setChassisSpeeds(s, ControlMode.CLOSED_LOOP_VELOCITY),
        new PPHolonomicDriveController(
            new PIDConstants(Translation.P, Translation.I, Translation.D),
            new PIDConstants(Rotation.P, Rotation.I, Rotation.D)),
        new RobotConfig(
            MASS.in(Kilograms),
            MOI.in(KilogramSquareMeters),
            new ModuleConfig(
                WHEEL_RADIUS,
                MAX_SPEED,
                WHEEL_COF,
                DCMotor.getNEO(1).withReduction(Driving.GEARING),
                Driving.STATOR_LIMIT,
                1),
            MODULE_OFFSET),
        () -> false,
        drive);

    NamedCommands.registerCommand(
        "elevator L4", new ScheduleCommand(elevator.scoreLevel(Level.L4)));

    NamedCommands.registerCommand(
        "score L4", new ScheduleCommand(scoraling.scoral(Level.L4)).withTimeout(1));

    NamedCommands.registerCommand("intake", new ScheduleCommand(scoraling.hpsIntake()));

    NamedCommands.registerCommand("retract", new ScheduleCommand(scoraling.retract()));

    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    chooser.addOption("no auto", Commands.none());
    chooser.addOption("B4 - alignment", B4(alignment, scoraling));
    chooser.addOption("P4 - alignment", P4(alignment, scoraling));
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

    Command auto = Commands.none();
    for (int i = 0; i < branches.size(); i++) {
      auto =
          Commands.sequence(
              auto,
              alignReef(branches.get(i), alignment, scoraling),
              alignSource(alignment, scoraling, 1));
    }

    return auto;
  }

  /**
   * Aligns to the reef with appropriate timeouts. It will end if it does not possess a coral.
   *
   * @param branch the branch to score on.
   */
  public static Command alignReef(Branch branch, Alignment alignment, Scoraling scoraling) {
    return alignment
        .reef(Level.L4, branch)
        .withTimeout(5)
        .onlyIf(() -> !scoraling.scoralBeambreak())
        .asProxy();
  }

  /**
   * Aligns to the source with appropriate timeouts. It will end if it already has a coral. It will
   * retry intaking a specified number of times.
   *
   * @param retries the number of times to retry (0 indicates to only run the command once)
   */
  public static Command alignSource(Alignment alignment, Scoraling scoraling, int retries) {
    Command source =
        alignment.source().andThen(scoraling.hpsIntake().withTimeout(2.5)).withTimeout(5);

    for (int i = 0; i < retries; i++) {
      source =
          source.andThen(
              alignment.source().andThen(scoraling.hpsIntake().withTimeout(2.5)).withTimeout(5));
    }

    source = Commands.race(source, Commands.waitUntil(() -> !scoraling.scoralBeambreak()));

    return source.onlyIf(() -> scoraling.scoralBeambreak()).asProxy();
  }

  /** Runas a "bottom" side auto with 4 L4 coral scored. */
  public static Command B4(Alignment alignment, Scoraling scoraling) {
    return alignAuto(alignment, scoraling, List.of(Branch.I, Branch.K, Branch.L, Branch.J));
  }

  /** runs a proceser side auto with 4 L4 coral scored. */
  public static Command P4(Alignment alignment, Scoraling scoraling) {
    return alignAuto(alignment, scoraling, List.of(Branch.E, Branch.D, Branch.C, Branch.B));
  }
}
