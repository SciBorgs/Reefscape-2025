package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.Constants.Field.TO_THE_LEFT;
import static org.sciborgs1155.robot.Constants.Field.moveForward;
import static org.sciborgs1155.robot.Constants.Field.strafe;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.RepulsorFieldPlanner;
import org.sciborgs1155.robot.Constants.Field.Branch;
import org.sciborgs1155.robot.Constants.Field.Face;
import org.sciborgs1155.robot.Constants.Field.Face.Side;
import org.sciborgs1155.robot.Constants.Field.Source;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import org.sciborgs1155.robot.scoral.Scoral;

public class Alignment implements Logged {
  @IgnoreLogged private final Drive drive;
  @IgnoreLogged private final Elevator elevator;
  @IgnoreLogged private final Scoral scoral;

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
    Pose2d goal = branch.withLevel(level).transformBy(strafe(TO_THE_LEFT.times(-1)));
    return Commands.sequence(
            pathfind(goal).asProxy(),
            Commands.parallel(
                elevator.scoreLevel(level).asProxy(),
                Commands.sequence(
                    drive.driveTo(goal).asProxy(),
                    Commands.waitUntil(elevator::atGoal)
                        .andThen(scoral.score().asProxy().until(scoral.beambreakTrigger)),
                    drive.driveTo(moveForward(goal, Meters.of(-0.2))).asProxy())))
        .withName("align to reef");
  }

  /**
   * Pathfinds and aligns to a designated source.
   *
   * @param source The source to pathfind to.
   * @return A command to go to a source.
   */
  public Command source(Source source) {
    return Commands.defer(
        () -> alignTo(source.pose).deadlineFor(elevator.retract()), Set.of(drive, elevator));
  }

  /**
   * Pathfinds and aligns to the nearest source.
   *
   * @return A command to go to the nearest source.
   */
  public Command source() {
    return Commands.defer(
        () ->
            alignTo(
                    drive
                        .pose()
                        .nearest(
                            Arrays.stream(Source.values())
                                .map(s -> s.pose)
                                .collect(Collectors.toList())))
                .deadlineFor(elevator.retract()),
        Set.of(drive, elevator));
  }

  public Command alignTo(Pose2d goal) {
    return pathfind(goal).andThen(drive.driveTo(goal));
  }

  /**
   * Finds the nearest reef face, then pathfinds to the branch with a given side on that face, and
   * scores on a designated level on that branch.
   *
   * @param level The level (L1, L2, L3, L4) being scored on.
   * @param side The branch side (Left/Right) to score on.
   * @return A command to score on the nearest reef branch.
   */
  public Command nearReef(Level level, Side side) {
    return Commands.deferredProxy(() -> reef(level, Face.nearest(drive.pose()).branch(side)));
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
    return Commands.defer(
            () ->
                drive
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
                    .until(() -> drive.atTranslation(goal.getTranslation(), Meters.of(1))),
            Set.of(drive))
        .withName("pathfind");
  }

  // @Log.NT public Pose2d abl = Face.AB.left.withLevel(Level.L4);
  // @Log.NT public Pose2d cdl = Face.CD.left.withLevel(Level.L4);
  // @Log.NT public Pose2d efl = Face.EF.left.withLevel(Level.L4);
  // @Log.NT public Pose2d ghl = Face.GH.left.withLevel(Level.L4);
  // @Log.NT public Pose2d ijl = Face.IJ.left.withLevel(Level.L4);
  // @Log.NT public Pose2d kll = Face.KL.left.withLevel(Level.L4);
  // @Log.NT public Pose2d abr = Face.AB.right.withLevel(Level.L4);
  // @Log.NT public Pose2d cdr = Face.CD.right.withLevel(Level.L4);
  // @Log.NT public Pose2d efr = Face.EF.right.withLevel(Level.L4);
  // @Log.NT public Pose2d ghr = Face.GH.right.withLevel(Level.L4);
  // @Log.NT public Pose2d ijr = Face.IJ.right.withLevel(Level.L4);
  // @Log.NT public Pose2d klr = Face.KL.right.withLevel(Level.L4);

  @Log.NT public Pose2d leftSourceLeft = Source.LEFT_SOURCE_LEFT.pose;
  @Log.NT public Pose2d leftSourceMid = Source.LEFT_SOURCE_MID.pose;
  @Log.NT public Pose2d leftSourceRight = Source.LEFT_SOURCE_RIGHT.pose;
  @Log.NT public Pose2d rightSourceLeft = Source.RIGHT_SOURCE_LEFT.pose;
  @Log.NT public Pose2d rightSourceMid = Source.RIGHT_SOURCE_MID.pose;
  @Log.NT public Pose2d rightSourceRight = Source.RIGHT_SOURCE_RIGHT.pose;
}
