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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Arrays;
import java.util.function.Consumer;
import org.sciborgs1155.robot.FieldConstants.Branch;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.elevator.*;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import org.sciborgs1155.robot.scoral.Scoral;

public class Autos {
  public static SendableChooser<Command> configureAutos(
      Drive drive, Scoraling scoraling, Elevator elevator, Alignment alignment, Scoral scoral) {
    AutoBuilder.configure(
        drive::pose,
        drive::resetOdometry,
        drive::robotRelativeChassisSpeeds,
        (s, g) -> drive.setChassisSpeeds(s, ControlMode.CLOSED_LOOP_VELOCITY, elevator::position),
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
    chooser.addOption(
        "B4 - alignment", RB4(alignment, scoraling, drive::resetOdometry, scoral, elevator));
    chooser.addOption("P4", RP4(alignment, scoraling, drive::resetOdometry));
    chooser.addOption(
        "line",
        drive.run(
            () ->
                drive.setChassisSpeeds(
                    new ChassisSpeeds(0.5, 0, 0),
                    ControlMode.OPEN_LOOP_VELOCITY,
                    elevator::position)));
    chooser.addOption("practice field", test(alignment, scoraling, drive::resetOdometry));

    return chooser;
  }

  public static Command sequence(Command... commands) {
    return Arrays.stream(commands)
        .map(c -> (Command) c.asProxy())
        .reduce(Commands.none(), (a, b) -> a.andThen(b));
  }

  public static Command RB4(
      Alignment alignment,
      Scoraling scoraling,
      Consumer<Pose2d> resetOdometry,
      org.sciborgs1155.robot.scoral.Scoral scoral,
      Elevator elevator) {
    return Commands.sequence(
        alignment.reef(Level.L4, Branch.I).withTimeout(4).asProxy(),
        // .onlyIf(() -> !scoraling.scoralBeambreak()),
        alignment
            .source()
            .asProxy()
            .withTimeout(5)
            .andThen(scoraling.hpsIntake().withTimeout(1).asProxy()),
        // .onlyIf(() -> scoraling.scoralBeambreak()),
        alignment
            .reef(Level.L4, Branch.K)
            .withTimeout(4)
            .asProxy(), // .onlyIf(() -> !scoraling.scoralBeambreak()),
        alignment
            .source()
            .asProxy()
            .withTimeout(8)
            .andThen(scoraling.hpsIntake().asProxy().withTimeout(5)),
        // .asProxy().onlyIf(() ->
        // scoraling.scoralBeambreak()),
        alignment
            .reef(Level.L4, Branch.L)
            .withTimeout(5)
            .asProxy(), // .onlyIf(() -> !scoraling.scoralBeambreak()),
        alignment
            .source()
            .asProxy()
            .withTimeout(8)
            .andThen(scoraling.hpsIntake().asProxy().withTimeout(5)), // .asProxy().onlyIf(() ->
        // scoraling.scoralBeambreak()),
        alignment
            .reef(Level.L4, Branch.J)
            .asProxy()
            .withTimeout(5)
            .asProxy()); // .onlyIf(() -> !scoraling.scoralBeambreak());
  }

  public static Command test(
      Alignment alignment, Scoraling scoraling, Consumer<Pose2d> resetOdometry) {
    return Commands.sequence(alignment.reef(Level.L4, Branch.G).withTimeout(4).asProxy());
    // .onlyIf(() -> !scoraling.scoralBeambreak()),
    //   alignment
    //       .source()
    //       .asProxy()
    //       .withTimeout(5)
    //       .andThen(scoraling.hpsIntake().withTimeout(1).asProxy()));
    // .onlyIf(() -> scoraling.scoralBeambreak()),
    //   alignment
    //       .reef(Level.L4, Branch.K)
    //       .withTimeout(5)
    //       .asProxy(), // .onlyIf(() -> !scoraling.scoralBeambreak()),
    //   alignment
    //       .source()
    //       .asProxy()
    //       .withTimeout(8)
    //       .andThen(scoraling.hpsIntake().asProxy().withTimeout(5)),
    //   // .asProxy().onlyIf(() ->
    //   // scoraling.scoralBeambreak()),
    //   alignment
    //       .reef(Level.L4, Branch.L)
    //       .withTimeout(5)
    //       .asProxy(), // .onlyIf(() -> !scoraling.scoralBeambreak()),
    //   alignment
    //       .source()
    //       .asProxy()
    //       .withTimeout(8)
    //       .andThen(scoraling.hpsIntake().asProxy().withTimeout(5)), // .asProxy().onlyIf(() ->
    // scoraling.scoralBeambreak()),
    //   alignment
    //       .reef(Level.L4, Branch.J)
    //       .asProxy()
    //       .withTimeout(5)
    //       .asProxy()); // .onlyIf(() -> !scoraling.scoralBeambreak());
  }

  public static Command RP4(
      Alignment alignment, Scoraling scoraling, Consumer<Pose2d> resetOdometry) {
    return Commands.sequence(
        alignment.reef(Level.L4, Branch.E).withTimeout(4).asProxy(),
        // .onlyIf(() -> !scoraling.scoralBeambreak()),
        alignment
            .source()
            .asProxy()
            .withTimeout(5)
            .andThen(scoraling.hpsIntake().withTimeout(1).asProxy()),
        // .onlyIf(() -> scoraling.scoralBeambreak()),
        alignment
            .reef(Level.L4, Branch.D)
            .withTimeout(5)
            .asProxy(), // .onlyIf(() -> !scoraling.scoralBeambreak()),
        alignment
            .source()
            .asProxy()
            .withTimeout(8)
            .andThen(scoraling.hpsIntake().asProxy().withTimeout(5)),
        // .asProxy().onlyIf(() ->
        // scoraling.scoralBeambreak()),
        alignment
            .reef(Level.L4, Branch.C)
            .withTimeout(5)
            .asProxy(), // .onlyIf(() -> !scoraling.scoralBeambreak()),
        alignment
            .source()
            .asProxy()
            .withTimeout(8)
            .andThen(scoraling.hpsIntake().asProxy().withTimeout(5)), // .asProxy().onlyIf(() ->
        // scoraling.scoralBeambreak()),
        alignment
            .reef(Level.L4, Branch.B)
            .asProxy()
            .withTimeout(5)
            .asProxy()); // .onlyIf(() -> !scoraling.scoralBeambreak());
  }
}
