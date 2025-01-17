package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.Constants.Robot.MASS;
import static org.sciborgs1155.robot.Constants.Robot.MOI;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import org.sciborgs1155.robot.Constants.Field.Branch;
import org.sciborgs1155.robot.Constants.Field.Level;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
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
    return pathfind(branch.pose)
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
    return pathfind(branch.pose)
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
    return pathfind(drive.pose().nearest(List.of(LEFT_SOURCE, RIGHT_SOURCE)))
        .alongWith(elevator.retract());
  }

  public Command processor() {
    return pathfind(PROCESSOR);
    // TODO idk how this would work. research this a bit more
  }

  public Command cage() {
    return pathfind(nearestCage(drive.pose()));
    // TODO it should do more. when we have a plan, update this
  }

  /**
   * Creates a pathplanner path to a goal (while avoiding obstacles) and then follows it.
   *
   * @param goal The goal ending pose for the robot.
   * @return A command to pathfind and drive to a goal pose.
   */
  public Command pathfind(Pose2d goal) {
    PathConstraints constraints =
        new PathConstraints(MAX_SPEED, MAX_ACCEL, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCEL);
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drive.pose(), goal);
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0, goal.getRotation()));
    path.preventFlipping = true;
    return new FollowPathCommand(
      path,
      drive::pose,
      drive::robotRelativeChassisSpeeds,
      (ChassisSpeeds a, DriveFeedforwards b) -> drive.setChassisSpeeds(a, DRIVE_MODE),
      new PPHolonomicDriveController(
          new PIDConstants(Translation.P, Translation.I, Translation.D),
          new PIDConstants(Rotation.P, Rotation.I, Rotation.D)),
      new RobotConfig(
          MASS,
          MOI,
          new ModuleConfig(
              WHEEL_RADIUS,
              MAX_SPEED,
              WHEEL_COF,
              DCMotor.getKrakenX60(1),
              DriveConstants.ModuleConstants.Driving.GEARING,
              DriveConstants.ModuleConstants.Driving.CURRENT_LIMIT,
              1),
          DriveConstants.TRACK_WIDTH),
      () -> false,
      drive);
  }
}
