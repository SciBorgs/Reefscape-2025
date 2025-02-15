package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.lib.RepulsorFieldPlanner;
import org.sciborgs1155.robot.Constants.Field.Branch;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import org.sciborgs1155.robot.scoral.Scoral;

public class Alignment {
  private Drive drive;
  private Elevator elevator;
  private Scoral scoral;

  private RepulsorFieldPlanner planner = new RepulsorFieldPlanner();

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
                    .andThen(scoral.score())))
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
        .deadlineFor(
            Commands.waitUntil(() -> elevator.atPosition(level.extension.in(Meters)))
                .andThen(scoral.score()));
  }

  /**
   * Pathfinds around obstacles and drives to a certain pose on the field.
   *
   * @param goal The field pose to pathfind to.
   * @return A Command to pathfind to an onfield pose.
   */
  public Command pathfind(Pose2d goal) {
    return drive
        .run(
            () -> {
              planner.setGoal(goal.getTranslation());
              drive.goToSample(
                  planner.getCmd(
                      drive.pose(),
                      drive.fieldRelativeChassisSpeeds(),
                      DriveConstants.MAX_SPEED.in(MetersPerSecond),
                      true),
                  goal.getRotation());
            })
        .until(
            () ->
                drive.pose().relativeTo(goal).getTranslation().getNorm()
                    < DriveConstants.Translation.TOLERANCE.times(3).in(Meters))
        .andThen(drive.driveTo(goal));
  }
}
