package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.Constants.advance;
import static org.sciborgs1155.robot.FieldConstants.allianceFromPose;
import static org.sciborgs1155.robot.FieldConstants.nearestBarge;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Arrays;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;
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
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import org.sciborgs1155.robot.led.LEDs;
import org.sciborgs1155.robot.scoral.Scoral;

public class Alignment {
  @NotLogged private final Drive drive;
  @NotLogged private final Elevator elevator;
  @NotLogged private final Scoral scoral;
  @NotLogged private final LEDs leds;

  private RepulsorFieldPlanner planner = new RepulsorFieldPlanner();

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
    Supplier<Pose2d> goal = () -> branch.withLevel(level);
    return Commands.sequence(
            Commands.runOnce(
                    () ->
                        Epilogue.getConfig()
                            .backend
                            .log("/Robot/alignment/goal pose", goal.get(), Pose2d.struct))
                .asProxy(),
            pathfind(() -> goal.get().transformBy(advance(Meters.of(-0.5))))
                .withName("go to reef")
                .asProxy(),
            // pathfind(goal, MetersPerSecond.of(0.1))
            //     .withTimeout(0.2)
            //     .withName("approach slowly")
            //     .asProxy(),
            Commands.deadline(
                Commands.sequence(
                    drive.driveTo(goal).asProxy().withTimeout(4),
                    Commands.waitUntil(elevator::atGoal)
                        .withTimeout(1.5)
                        .andThen(scoral.score().asProxy().until(scoral.blocked.negate())),
                    moveRobotRelative(advance(Meters.of(-0.2))).asProxy()),
                elevator.scoreLevel(level).asProxy(),
                leds.error(
                    () ->
                        drive
                                .pose()
                                .relativeTo(goal.get())
                                .getTranslation()
                                .getDistance(Translation2d.kZero)
                            * 3.5,
                    0.02 * 3.5)))
        .asProxy()
        .withName("align to reef")
        .onlyWhile(
            () ->
                !FaultLogger.report(
                    allianceFromPose(goal.get()) != allianceFromPose(drive.pose()),
                    alternateAlliancePathfinding));
  }

  public Command weirdReef(Level level, Branch branch) {
    Supplier<Pose2d> goal = () -> branch.withLevel(level);
    return Commands.sequence(
            Commands.runOnce(
                    () ->
                        Epilogue.getConfig()
                            .backend
                            .log("/Robot/alignment/goal pose", goal.get(), Pose2d.struct))
                .asProxy(),
            pathfind(() -> goal.get().transformBy(advance(Meters.of(-0.5))))
                .withName("go to reef")
                .asProxy(),
            Commands.deadline(
                Commands.sequence(
                    drive.newDriveTo(goal).asProxy().withTimeout(4),
                    Commands.waitUntil(elevator::atGoal)
                        .withTimeout(1.5)
                        .andThen(scoral.score().asProxy().until(scoral.blocked.negate())),
                    moveRobotRelative(advance(Meters.of(-0.2))).asProxy()),
                elevator.scoreLevel(level).asProxy(),
                leds.error(
                    () ->
                        drive
                                .pose()
                                .relativeTo(goal.get())
                                .getTranslation()
                                .getDistance(Translation2d.kZero)
                            * 3.5,
                    0.02 * 3.5)))
        .asProxy()
        .withName("new weird align to reef")
        .onlyWhile(
            () ->
                !FaultLogger.report(
                    allianceFromPose(goal.get()) != allianceFromPose(drive.pose()),
                    alternateAlliancePathfinding));
  }

  public Command moveRobotRelative(Transform2d transform) {
    return Commands.defer(
        () -> {
          Pose2d goal = drive.pose().transformBy(transform);
          return drive.driveTo(goal);
        },
        Set.of(drive));
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
   * Pathfinds and aligns to the nearest barge position.
   *
   * @return A command to align to the barge.
   */
  public Command barge() {
    return alignTo(() -> nearestBarge(drive.pose())).asProxy();
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

  public Command alignTo(Supplier<Pose2d> goal) {
    return Commands.runOnce(
            () ->
                Epilogue.getConfig()
                    .backend
                    .log("/Robot/alignment/goal pose", goal.get(), Pose2d.struct))
        .andThen(
            pathfind(goal, Meters.of(1))
                .asProxy()
                .andThen(
                    drive
                        .driveTo(goal)
                        .asProxy()
                        .deadlineFor(
                            leds.error(
                                () ->
                                    drive
                                            .pose()
                                            .getTranslation()
                                            .getDistance(goal.get().getTranslation())
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
  public Command nearReef(Side side, Level level) {
    return alignTo(() -> Face.nearest(drive.pose()).branch(side).withLevel(level)).asProxy();
  }

  /**
   * Finds the nearest reef face, then pathfinds right up to it.
   *
   * @return A Command to align to the nearest reef branch.
   */
  public Command nearAlgae() {
    return alignTo(() -> Face.nearest(drive.pose()).pose()).asProxy();
  }

  /**
   * Pathfinds around obstacles and drives to a certain pose on the field.
   *
   * @param goal The field pose to pathfind to.
   * @param maxSpeed The maximum speed the path will command the drivetrain to.
   * @return A Command to pathfind to an onfield pose.
   */
  public Command pathfind(Supplier<Pose2d> goal, LinearVelocity maxSpeed, Distance tolerance) {
    double speed = maxSpeed.in(MetersPerSecond);
    return drive
        .run(
            () -> {
              Tracer.startTrace("repulsor pathfinding");
              planner.setGoal(goal.get().getTranslation());
              drive.goToSample(
                  planner.getCmd(drive.pose(), drive.fieldRelativeChassisSpeeds(), speed, true),
                  goal.get().getRotation(),
                  elevator::position);
              Tracer.endTrace();
            })
        .until(() -> drive.atTranslation(goal.get().getTranslation(), tolerance))
        .onlyWhile(
            () ->
                !FaultLogger.report(
                    allianceFromPose(goal.get()) != allianceFromPose(drive.pose()),
                    alternateAlliancePathfinding))
        .withName("pathfind");
  }

  /**
   * Pathfinds around obstacles and drives to a certain pose on the field.
   *
   * @param goal The field pose to pathfind to.
   * @return A Command to pathfind to an onfield pose.
   */
  public Command pathfind(Supplier<Pose2d> goal, Distance tolerance) {
    return pathfind(goal, DriveConstants.MAX_SPEED.times(0.7), tolerance);
  }

  /**
   * Pathfinds around obstacles and drives to a certain pose on the field.
   *
   * @param goal The field pose to pathfind to.
   * @return A Command to pathfind to an onfield pose.
   */
  public Command pathfind(Supplier<Pose2d> goal, LinearVelocity maxSpeed) {
    return pathfind(goal, maxSpeed, Translation.TOLERANCE);
  }

  /**
   * Pathfinds around obstacles and drives to a certain pose on the field.
   *
   * @param goal The field pose to pathfind to.
   * @return A Command to pathfind to an onfield pose.
   */
  public Command pathfind(Supplier<Pose2d> goal) {
    return pathfind(goal, Translation.TOLERANCE);
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
    return pathfind(() -> drive.pose(), MetersPerSecond.of(0))
        .withTimeout(3)
        .andThen(() -> System.out.println("[Alignment] Finished warmup"))
        .ignoringDisable(true);
  }
}
