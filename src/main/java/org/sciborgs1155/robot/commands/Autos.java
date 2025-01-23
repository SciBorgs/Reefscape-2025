package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.drive.DriveConstants.ROBOT_CONFIG;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;

public class Autos {
  // private static Optional<Rotation2d> rotation = Optional.empty();

  public static SendableChooser<Command> configureAutos(Drive drive) {
    AutoBuilder.configure(
        drive::pose,
        drive::resetOdometry,
        drive::robotRelativeChassisSpeeds,
        (s, g) -> drive.setChassisSpeeds(s, ControlMode.CLOSED_LOOP_VELOCITY),
        new PPHolonomicDriveController(
            new PIDConstants(Translation.P, Translation.I, Translation.D),
            new PIDConstants(Rotation.P, Rotation.I, Rotation.D)),
        ROBOT_CONFIG,
        () -> true,
        drive);

    // PPHolonomicDriveController.overrideRotationFeedback(() -> rotation.get().getRadians());

    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    chooser.addOption("no auto", Commands.none());
    return chooser;
  }
}
