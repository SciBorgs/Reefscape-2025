package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import org.sciborgs1155.robot.Constants.Field.Branch;
import org.sciborgs1155.robot.Constants.Field.Level;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.elevator.Elevator;
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
    return pathfollow(pathfind(branch.pose))
        .alongWith(elevator.scoreLevel(level).until(() -> elevator.atPosition(level.height)))
        .andThen(scoral.outtake());
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
    return pathfollow(pathfind(branch.pose))
        .andThen(elevator.scoreLevel(level).until(() -> elevator.atPosition(level.height)))
        .andThen(scoral.outtake());
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
    return pathfollow(pathfind(drive.pose().nearest(List.of(LEFT_SOURCE, RIGHT_SOURCE))))
        .alongWith(elevator.retract());
  }

  public Command processor() {
    return pathfollow(pathfind(PROCESSOR));
    // TODO idk how this would work. research this a bit more
  }

  public Command cage() {
    return pathfollow(pathfind(nearestCage(drive.pose())));
    // TODO it should do more. when we have a plan, update this
  }

  /**
   * Creates a pathplanner path to a goal (while avoiding obstacles).
   *
   * @param goal The goal ending pose for the robot.
   * @return A pathplanner path to get to a field position.
   */
  public PathPlannerPath pathfind(Pose2d goal) {
    PathConstraints constraints =
        new PathConstraints(MAX_SPEED, MAX_ACCEL, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCEL);
    var speeds = drive.robotRelativeChassisSpeeds();
    double currentSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drive.pose(), goal);
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            new IdealStartingState(currentSpeed, drive.heading()),
            new GoalEndState(0, goal.getRotation()));
    path.preventFlipping = true;
    return path;
  }

  /**
   * Follows a given pathplanner path.
   *
   * @param path A pathplanner path.
   * @return A command to follow a path.
   */
  public Command pathfollow(PathPlannerPath path) {
    System.out.println(
        "Waypoint 1: "
            + path.getWaypoints().get(path.getWaypoints().size() - 2).anchor().toString());
    System.out.println(
        "Waypoint 2: "
            + path.getWaypoints().get(path.getWaypoints().size() - 1).anchor().toString());
    System.out.println(
        path.generateTrajectory(drive.robotRelativeChassisSpeeds(), drive.heading(), ROBOT_CONFIG)
            .getTotalTimeSeconds());

    return new FollowPathCommand(
        path,
        drive::pose,
        drive::robotRelativeChassisSpeeds,
        (ChassisSpeeds a, DriveFeedforwards b) ->
            drive.setChassisSpeeds(a, ControlMode.CLOSED_LOOP_VELOCITY),
        new PPHolonomicDriveController(
            new PIDConstants(Translation.P, Translation.I, Translation.D),
            new PIDConstants(Rotation.P, Rotation.I, Rotation.D)),
        ROBOT_CONFIG,
        () -> false,
        drive);
  }
}
