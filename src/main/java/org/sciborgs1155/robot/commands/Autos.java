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
    chooser.addOption("RB4 - alignment", B4(alignment, scoraling));
    chooser.addOption("testy stuffy", testyStuffy(alignment, scoraling));
    return chooser;
  }

  /** Warning that your list will be desecrated */
  public static Command alignAuto(
      Alignment alignment, Scoraling scoraling, LinkedList<Branch> branches) {
    if (branches.isEmpty()) {
      FaultLogger.report(
          new Fault("alignAuto fault", "alignAuto passed zero branches", FaultType.WARNING));
      return Commands.none();
    }

    Command part =
        alignment
            .reef(Level.L4, branches.getFirst())
            .withTimeout(5)
            .onlyIf(() -> !scoraling.scoralBeambreak())
            .asProxy();

    if (branches.size() > 1) {
      part =
          part.andThen(
              alignment
                  .source()
                  .andThen(scoraling.hpsIntake())
                  .withTimeout(5)
                  .onlyIf(() -> scoraling.scoralBeambreak())
                  .asProxy());
    }

    branches.removeFirst();

    return part.andThen(alignAuto(alignment, scoraling, branches));
  }

  public static Command testyStuffy(Alignment alignment, Scoraling scoraling) {
    return alignAuto(
        alignment, scoraling, new LinkedList<Branch>(Arrays.asList(Branch.A, Branch.B, Branch.C)));
  }

  public static Command B4(Alignment alignment, Scoraling scoraling) {
    return alignAuto(
        alignment,
        scoraling,
        new LinkedList<Branch>(Arrays.asList(Branch.I, Branch.K, Branch.L, Branch.J)));
  }

  public static Command P4(Alignment alignment, Scoraling scoraling) {
    return alignAuto(
        alignment,
        scoraling,
        new LinkedList<Branch>(Arrays.asList(Branch.E, Branch.D, Branch.C, Branch.B)));
  }

  /**
   * Pathfinds and aligns to the nearest source.
   *
   * @param retries the number of times to retry (0 means it runs and never retries)
   * @return A command to go to the nearest source.
   */
  public Command source(Alignment alignment, Scoraling scoraling, int retries) {
    Command source = alignment.source();

    for (int i = 0; i < retries; i++) {
        source = source.andThen(alignment.source());
    }

    return Commands.race(
        alignment.source(), Commands.waitUntil(() -> !scoraling.scoralBeambreak()));
  }
}
