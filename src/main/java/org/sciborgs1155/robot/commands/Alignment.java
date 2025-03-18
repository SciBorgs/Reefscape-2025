package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.Constants.advance;
import static org.sciborgs1155.robot.FieldConstants.allianceFromPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.Fault;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.lib.RepulsorFieldPlanner;
import org.sciborgs1155.lib.Tracer;
import org.sciborgs1155.robot.FieldConstants.Branch;
import org.sciborgs1155.robot.FieldConstants.Face;
import org.sciborgs1155.robot.FieldConstants.Face.Side;
import org.sciborgs1155.robot.FieldConstants.Source;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import org.sciborgs1155.robot.led.LEDs;
import org.sciborgs1155.robot.scoral.Scoral;

public class Alignment implements Logged {
  @IgnoreLogged private final Drive drive;
  @IgnoreLogged private final Elevator elevator;
  @IgnoreLogged private final Scoral scoral;
  @IgnoreLogged private final LEDs leds;

  @Log.NT private RepulsorFieldPlanner planner = new RepulsorFieldPlanner();

  private Fault alternateAlliancePathfinding =
      new Fault(
          "Alternate Alliance Pathfinding",
          "The robot is attempting to pathfind to a pose on the other alliance.",
          FaultType.WARNING);

  /**
   * Constructor for an Alignment command object.
   *
   * @param drive The operated drivetrain.
   * @param elevator The operated elevator.
   */
  public Alignment(Drive drive, Elevator elevator, Scoral scoral, LEDs leds) {
    this.drive = drive;
    this.elevator = elevator;
    this.scoral = scoral;
    this.leds = leds;
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
    Supplier<Pose2d> goal = branch::pose;
    return Commands.sequence(
            Commands.runOnce(() -> log("goal pose", goal.get())).asProxy(),
            pathfind(goal).withName("").asProxy(),
            Commands.parallel(
                elevator.scoreLevel(level).asProxy(),
                leds.error(
                    () ->
                        drive
                                .pose()
                                .relativeTo(goal.get())
                                .getTranslation()
                                .getDistance(new Translation2d())
                            * 3,
                    0.02 * 3),
                Commands.sequence(
                    drive.driveTo(goal).asProxy().withTimeout(4),
                    Commands.waitUntil(elevator::atGoal)
                        .withTimeout(1.5)
                        .andThen(scoral.score(level).asProxy().until(scoral.blocked.negate())),
                    drive
                        .driveTo(() -> goal.get().transformBy(advance(Meters.of(-0.2))))
                        .asProxy())))
        .withName("align to reef")
        .onlyWhile(
            () ->
                !FaultLogger.report(
                    allianceFromPose(goal.get()) != allianceFromPose(drive.pose()),
                    alternateAlliancePathfinding));
  }

  /**
   * Pathfinds and aligns to a designated source.
   *
   * @param source The source to pathfind to.
   * @return A command to go to a source.
   */
  public Command source(Source source) {
    return alignTo(source::pose).deadlineFor(elevator.retract()).asProxy();
  }

  /**
   * Pathfinds and aligns to the nearest source.
   *
   * @return A command to go to the nearest source.
   */
  public Command source() {
    return alignTo(
            () ->
                drive
                    .pose()
                    .nearest(
                        Arrays.stream(Source.values())
                            .map(s -> s.pose())
                            .collect(Collectors.toList())))
        .deadlineFor(elevator.retract())
        .asProxy();
  }

  /** THIS COMMAND IS UNDEFERRED. */
  public Command alignTo(Supplier<Pose2d> goal) {
    return Commands.runOnce(() -> log("goal pose", goal.get()))
        .asProxy()
        .andThen(
            pathfind(goal)
                .andThen(
                    drive
                        .driveTo(goal)
                        .alongWith(
                            leds.error(
                                () ->
                                    drive
                                            .pose()
                                            .relativeTo(goal.get())
                                            .getTranslation()
                                            .getDistance(new Translation2d())
                                        * 3,
                                0.02 * 3)))
                .onlyWhile(
                    () ->
                        !FaultLogger.report(
                            allianceFromPose(goal.get()) != allianceFromPose(drive.pose()),
                            alternateAlliancePathfinding)));
  }

  /**
   * Finds the nearest reef face, then pathfinds to the branch with a given side on that face, and
   * scores on a designated level on that branch.
   *
   * @param side The branch side (Left/Right) to score on.
   * @return A command to align to the nearest reef branch.
   */
  public Command nearReef(Side side) {
    return alignTo(() -> Face.nearest(drive.pose()).branch(side).pose()).asProxy();
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
    return pathfind(branch::pose)
        .andThen(elevator.scoreLevel(level))
        .until(scoral.blocked.negate())
        .deadlineFor(
            Commands.waitUntil(() -> elevator.atPosition(level.extension.in(Meters)))
                .andThen(scoral.score(level)));
  }

  /**
   * Pathfinds around obstacles and drives to a certain pose on the field.
   *
   * @param goal The field pose to pathfind to.
   * @return A Command to pathfind to an onfield pose.
   */
  public Command pathfind(Supplier<Pose2d> goal) {
    return drive
        .run(
            () -> {
              Tracer.startTrace("repulsor pathfinding");
              planner.setGoal(goal.get().getTranslation());
              drive.goToSample(
                  planner.getCmd(
                      drive.pose(),
                      drive.fieldRelativeChassisSpeeds(),
                      DriveConstants.MAX_SPEED.in(MetersPerSecond),
                      true),
                  goal.get().getRotation(),
                  elevator::position);
              Tracer.endTrace();
            })
        .until(() -> drive.atTranslation(goal.get().getTranslation(), Meters.of(1)))
        .onlyWhile(
            () ->
                !FaultLogger.report(
                    allianceFromPose(goal.get()) != allianceFromPose(drive.pose()),
                    alternateAlliancePathfinding))
        .withName("pathfind");
  }

  /**
   * Moves the robot (unobtrusively) around while slowing down when about to ram into a field
   * element.
   *
   * @param x Driver's vx input.
   * @param y Driver's vy input.
   * @param omega Driver's omega input.
   * @return A command to drive without worry of ramming into things.
   */
  public Command freeDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return drive.run(
        () -> {
          Tracer.startTrace("repulsor pathfinding");
          planner.setGoal(
              drive
                  .pose()
                  .getTranslation()
                  .plus(new Translation2d(x.getAsDouble(), y.getAsDouble())));
          // TODO this makes no sense! -- siggy
          drive.addOnSample(
              x,
              y,
              omega,
              planner.getCmd(
                  drive.pose(),
                  drive.fieldRelativeChassisSpeeds(),
                  DriveConstants.MAX_SPEED.in(MetersPerSecond),
                  true),
              elevator::position);
          Tracer.endTrace();
        });
  }

  // * Warms up the pathfind command by telling drive to drive to itself. */
  public Command warmupCommand() {
    return pathfind(() -> drive.pose())
        .withTimeout(5)
        .andThen(Commands.print("[Alignment] Finished warmup"))
        .ignoringDisable(true);
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

  // @Log.NT public Pose2d leftSourceLeft = Source.LEFT_SOURCE_LEFT.pose;
  // @Log.NT public Pose2d leftSourceMid = Source.LEFT_SOURCE_MID.pose;
  // @Log.NT public Pose2d leftSourceRight = Source.LEFT_SOURCE_RIGHT.pose;
  // @Log.NT public Pose2d rightSourceLeft = Source.RIGHT_SOURCE_LEFT.pose;
  // @Log.NT public Pose2d rightSourceMid = Source.RIGHT_SOURCE_MID.pose;
  // @Log.NT public Pose2d rightSourceRight = Source.RIGHT_SOURCE_RIGHT.pose;
}
