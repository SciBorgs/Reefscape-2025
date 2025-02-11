package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Set;
import org.sciborgs1155.robot.Constants.Field.Branch;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import org.sciborgs1155.robot.scoral.Scoral;

public class Alignment {
  private Drive drive;
  private Elevator elevator;
  private Scoral scoral;

  /**
   * Constructor for an Alignment command object.
   *
   * @param drive The operated drivetrain.
   * @param elevator The operated elevator.
   */
  public Alignment(Drive drive, Elevator elevator, Scoral scoral) {
    this.drive = drive;
    this.elevator = elevator;
    this.scoral = scoral;
  }

  /**
   * Drives to a designated reef branch while raising the elevator, and then scores onto a
   * designated level on that branch.
   *
   * @param level The level (L1, L2, L3, L4) being scored on.
   * @param branch The reef branch (A, B, C, etc.) being scored on.
   * @return A command to quickly prepare and then score in the reef.
   */
  public Command reef(Level level, Branch branch) {
    return (pathfind(branch.pose)
            .andThen(
                Commands.waitUntil(() -> elevator.atPosition(level.extension.in(Meters)))
                    .andThen(
                        scoral.score().withTimeout(1)))) // timeout needed because no sim beambreak
        .deadlineFor(elevator.scoreLevel(level));
  }

  /**
   * Drives to a designated reef branch, then raises the elevator, and then scores onto a designated
   * level on that branch.
   *
   * @param level The level (L1, L2, L3, L4) being scored on.
   * @param branch The reef branch (A, B, C, etc.) being scored on.
   * @return A command to score in the reef without raising the elevator while moving.
   */
  public Command safeReef(Level level, Branch branch) {
    return pathfind(branch.pose)
        .andThen(elevator.scoreLevel(level))
        .until(scoral::beambreak)
        .withTimeout(1)
        .deadlineFor(
            Commands.waitUntil(() -> elevator.atPosition(level.extension.in(Meters)))
                .andThen(scoral.score()));
  }

  /**
   * Drives to the nearest reef branch while raising the elevator, and then scores onto a designated
   * level.
   *
   * @param level The level (L1, L2, L3, L4) being scored on.
   * @return A command to score in the nearest reef branch.
   */
  public Command nearReef(Level level) {
    return reef(level, Branch.nearest(drive.pose()));
  }

  /**
   * Drives to the nearest source and then retracts the elevator for intaking.
   *
   * @return A command to align with the human player station source.
   */
  public Command source() {
    return pathfind(drive.pose().nearest(List.of(LEFT_SOURCE, RIGHT_SOURCE)));
  }

  /**
   * Creates a pathplanner path to a goal.
   *
   * @param goal The goal ending pose for the robot.
   * @return A pathplanner path to get to a field position.
   */
  public PathPlannerPath directPath(Pose2d goal) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drive.pose(), goal);
    double speed =
        Math.hypot(
            drive.robotRelativeChassisSpeeds().vxMetersPerSecond,
            drive.robotRelativeChassisSpeeds().vyMetersPerSecond);
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            PATH_CONSTRAINTS,
            new IdealStartingState(speed, drive.heading()),
            new GoalEndState(0, goal.getRotation()));
    path.preventFlipping = false;
    return path;
  }

  /**
   * Creates a pathplanner path to a goal, and then follows that path.
   *
   * @param goal The goal pose of the robot at the end of the path.
   * @return A command to follow a path.
   */
  public Command directPathfollow(Pose2d goal) {
    return Commands.defer(
        () ->
            new FollowPathCommand(
                    directPath(goal),
                    drive::pose,
                    drive::robotRelativeChassisSpeeds,
                    (ChassisSpeeds a, DriveFeedforwards b) ->
                        drive.setChassisSpeeds(a, ControlMode.CLOSED_LOOP_VELOCITY),
                    new PPHolonomicDriveController(
                        new PIDConstants(Translation.P, Translation.I, Translation.D),
                        new PIDConstants(Rotation.P, Rotation.I, Rotation.D)),
                    ROBOT_CONFIG,
                    () -> false,
                    drive)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                .andThen(
                    drive.driveTo(
                        goal)), // pathplanner isnt precise enough so we gotta fix it ourselves
        // (just adjusts a little bit)
        Set.of(drive));
  }

  /**
   * Pathfinds the drivetrain around obstacles to an input Pose2d, finishing with an end velocity of
   * 0.
   *
   * @param goal The goal end pose of the pathfinding.
   * @return A Command to pathfind around obstacles to a goal pose.
   */
  public Command pathfind(Pose2d goal) {
    return AutoBuilder.pathfindToPose(goal, PATH_CONSTRAINTS, 0.).andThen(drive.driveTo(goal));
    // driveTo is used to slightly adjust since pathfindToPose is not precise enough.
  }
}
