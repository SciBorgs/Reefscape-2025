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
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.elevator.*;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;

public class Autos {
  public static SendableChooser<Command> configureAutos(Drive drive, Scoraling scoraling, Elevator elevator) {
    AutoBuilder.configure(
        drive::pose,
        drive::resetOdometry,
        drive::robotRelativeChassisSpeeds,
        s -> drive.setChassisSpeeds(s, ControlMode.CLOSED_LOOP_VELOCITY),
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
                Driving.CURRENT_LIMIT,
                1),
            MODULE_OFFSET),
        () -> false,
        drive);

    NamedCommands.registerCommand("elevator L4", new ScheduleCommand(elevator.scoreLevel(Level.L4)));

    NamedCommands.registerCommand("score L4", new ScheduleCommand(scoraling.scoral(Level.L4)));

    NamedCommands.registerCommand("intake", new ScheduleCommand(scoraling.hpsIntake()));

    NamedCommands.registerCommand("retract", new ScheduleCommand(scoraling.retract()));

    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    chooser.addOption("no auto", Commands.none());
    return chooser;
  }
}

