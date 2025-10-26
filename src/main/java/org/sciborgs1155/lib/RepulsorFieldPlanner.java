package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.Seconds;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.sciborgs1155.robot.Constants;

/**
 * Implements a repulsor field-based path planner for autonomous navigation. Uses artificial
 * potential fields to generate obstacle-avoiding trajectories.
 */
public class RepulsorFieldPlanner {
  abstract static class Obstacle {
    double strength = 1.0;
    boolean positive = true;

    public Obstacle(double strength, boolean positive) {
      this.strength = strength;
      this.positive = positive;
    }

    /**
     * Calculates the repulsive force from this obstacle at a given position.
     *
     * @param position The current position.
     * @param target The goal position.
     * @return The force vector at the given position.
     */
    public abstract Force getForceAtPosition(Translation2d position, Translation2d target);

    /**
     * Converts distance to force magnitude using inverse square law.
     *
     * @param distance The distance from the position to the obstacle.
     * @return The force magnitude at that distance.
     */
    protected double distToForceMag(double distance) {
      double forceMag = strength / (0.00001 + Math.pow(distance, 2));
      return forceMag * (positive ? 1 : -1);
    }

    /**
     * Converts distance to force magnitude with falloff.
     *
     * @param distance The distance from the position to the obstacle.
     * @param falloff The falloff distance.
     * @return The force magnitude with subtracted falloff.
     */
    protected double distToForceMag(double distance, double falloff) {
      double original = strength / (0.00001 + Math.pow(distance, 2));
      double falloffMag = strength / (0.00001 + Math.pow(falloff, 2));
      return Math.max(original - falloffMag, 0) * (positive ? 1 : -1);
    }
  }

  static class PointObstacle extends Obstacle {
    Translation2d location;
    double radius = 0.5;

    public PointObstacle(Translation2d location, double strength, boolean positive) {
      super(strength, positive);
      this.location = location;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      double distance = location.getDistance(position);
      if (distance > 4) {
        return new Force();
      }

      double outwardsMagnitude = distToForceMag(location.getDistance(position) - radius);
      Force initialForce = new Force(outwardsMagnitude, position.minus(location).getAngle());

      Rotation2d theta =
          target.minus(position).getAngle().minus(position.minus(location).getAngle());

      double tangentialMagnitude =
          outwardsMagnitude * Math.signum(Math.sin(theta.getRadians() / 2)) / 2;

      return initialForce
          .rotateBy(Rotation2d.kCCW_90deg)
          .div(initialForce.getNorm())
          .times(tangentialMagnitude)
          .plus(initialForce);
    }
  }

  static class CircleObstacle extends Obstacle {
    Translation2d location;
    double radius = 0.5;

    public CircleObstacle(
        Translation2d location, double strength, double radius, boolean positive) {
      super(strength, positive);
      this.location = location;
      this.radius = radius;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      Translation2d targetToLocation = location.minus(target);
      Translation2d sidewaysPoint =
          new Translation2d(1, targetToLocation.getAngle()).plus(location);

      double sidewaysMagnitude = distToForceMag(sidewaysPoint.getDistance(position));
      double outwardsMagnitude =
          distToForceMag(Math.max(0.01, location.getDistance(position) - radius));

      Force initialForce =
          new Force(
              outwardsMagnitude,
              position.minus(location).getNorm() > 1e-4
                  ? position.minus(location).getAngle()
                  : Rotation2d.kZero);

      Rotation2d sidewaysTheta =
          target.minus(position).getNorm() > 1e-4
              ? target.minus(position).getAngle().minus(position.minus(sidewaysPoint).getAngle())
              : Rotation2d.kZero;

      double sideways = sidewaysMagnitude * Math.signum(Math.sin(sidewaysTheta.getRadians()));
      Rotation2d sidewaysAngle = targetToLocation.getAngle().rotateBy(Rotation2d.kCCW_90deg);

      return new Force(sideways, sidewaysAngle).plus(initialForce);
    }
  }

  static class HorizontalObstacle extends Obstacle {
    double y;

    public HorizontalObstacle(double y, double strength, boolean positive) {
      super(strength, positive);
      this.y = y;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      return new Force(0, distToForceMag(y - position.getY(), 1));
    }
  }

  static class VerticalObstacle extends Obstacle {
    double x;

    public VerticalObstacle(double x, double strength, boolean positive) {
      super(strength, positive);
      this.x = x;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      return new Force(distToForceMag(x - position.getX(), 1), 0);
    }
  }

  private static final double GOAL_STRENGTH = 0.65;
  private static final double FIELD_LENGTH = 16.42;
  private static final double FIELD_WIDTH = 8.16;
  private static final double CONTROL_PERIOD = 0.02;
  private static final int MAX_TRAJECTORY_ITERATIONS = 400;

  public static final List<Obstacle> FIELD_OBSTACLES =
      List.of(
          new CircleObstacle(
              new Translation2d(4.49, 4), 0.6, Units.inchesToMeters(65.5 / 2.0), true),
          new CircleObstacle(
              new Translation2d(13.08, 4), 0.6, Units.inchesToMeters(65.5 / 2.0), true));

  public static final List<Obstacle> WALLS =
      List.of(
          new HorizontalObstacle(0.0, 0.5, true),
          new HorizontalObstacle(FIELD_WIDTH, 0.5, false),
          new VerticalObstacle(0.0, 0.5, true),
          new VerticalObstacle(FIELD_LENGTH, 0.5, false));

  private final List<Obstacle> fixedObstacles = new ArrayList<>();
  private Optional<Translation2d> goalPosition = Optional.empty();
  private SwerveSample previousSample;

  public double pathLength = 0;

  public RepulsorFieldPlanner() {
    fixedObstacles.addAll(FIELD_OBSTACLES);
    fixedObstacles.addAll(WALLS);
    this.previousSample = sample(Translation2d.kZero, Rotation2d.kZero, 0, 0, 0);
  }

  @Logged
  public Pose2d goal() {
    return new Pose2d(goalPosition.orElse(Translation2d.kZero), Rotation2d.kZero);
  }

  /**
   * Calculates the attractive force towards the goal.
   *
   * @param currentLocation Current location of the robot.
   * @param goal Target goal position.
   * @return The attractive force vector towards the goal.
   */
  Force getGoalForce(Translation2d currentLocation, Translation2d goal) {
    Translation2d displacement = goal.minus(currentLocation);
    if (displacement.getNorm() == 0) {
      return new Force();
    }

    Rotation2d direction = displacement.getAngle();
    double magnitude =
        GOAL_STRENGTH * (1 + 1.0 / (0.0001 + displacement.getNorm() * displacement.getNorm()));
    return new Force(magnitude, direction);
  }

  /**
   * Calculates the repulsive force from all walls.
   *
   * @param currentLocation Current location of the robot.
   * @param target Target goal position.
   * @return The combined repulsive force from walls.
   */
  Force getWallForce(Translation2d currentLocation, Translation2d target) {
    Force totalForce = Force.kZero;
    for (Obstacle obstacle : WALLS) {
      totalForce = totalForce.plus(obstacle.getForceAtPosition(currentLocation, target));
    }
    return totalForce;
  }

  /**
   * Calculates the repulsive force from all field obstacles.
   *
   * @param currentLocation Current location of the robot.
   * @param target Target goal position.
   * @return The combined repulsive force from obstacles.
   */
  Force getObstacleForce(Translation2d currentLocation, Translation2d target) {
    Force totalForce = Force.kZero;
    for (Obstacle obstacle : FIELD_OBSTACLES) {
      totalForce = totalForce.plus(obstacle.getForceAtPosition(currentLocation, target));
    }
    return totalForce;
  }

  /**
   * Calculates the total force from all sources.
   *
   * @param currentLocation Current location of the robot.
   * @param target Target goal position.
   * @return The total resultant force vector.
   */
  Force getForce(Translation2d currentLocation, Translation2d target) {
    return getGoalForce(currentLocation, target)
        .plus(getObstacleForce(currentLocation, target))
        .plus(getWallForce(currentLocation, target));
  }

  /**
   * Creates a {@link SwerveSample} from component values.
   *
   * @param position Position of the robot.
   * @param heading Heading of the robot.
   * @param velocityX X-component of the field-relative robot velocity.
   * @param velocityY Y-component of the field-relative robot velocity.
   * @param angularVelocity Angular velocity of the robot.
   * @return A Choreo SwerveSample.
   */
  public static SwerveSample sample(
      Translation2d position,
      Rotation2d heading,
      double velocityX,
      double velocityY,
      double angularVelocity) {
    return new SwerveSample(
        0,
        position.getX(),
        position.getY(),
        heading.getRadians(),
        velocityX,
        velocityY,
        angularVelocity,
        0,
        0,
        0,
        new double[4],
        new double[4]);
  }

  /**
   * Sets the goal position for the path planner.
   *
   * @param goal The new goal position.
   */
  public void setGoal(Translation2d goal) {
    this.goalPosition = Optional.of(goal);
  }

  /**
   * Computes the next commanded state for the robot.
   *
   * @param pose Current pose of the robot.
   * @param currentSpeeds Current chassis speeds of the robot.
   * @param maxSpeed Desired maximum speed.
   * @param useGoal Whether to use the attractive goal force.
   * @return A SwerveSample representing the next desired state with obstacle avoidance.
   */
  public SwerveSample getCmd(
      Pose2d pose, ChassisSpeeds currentSpeeds, double maxSpeed, boolean useGoal) {
    return getCmd(pose, currentSpeeds, maxSpeed, useGoal, pose.getRotation());
  }

  /**
   * Computes the next commanded state for the robot with a specified goal rotation.
   *
   * @param pose Current pose of the robot.
   * @param currentSpeeds Current chassis speeds of the robot.
   * @param maxSpeed Desired maximum speed.
   * @param useGoal Whether to use the attractive goal force.
   * @param goalRotation Desired rotation at the goal.
   * @return A SwerveSample representing the next desired state with obstacle avoidance.
   */
  public SwerveSample getCmd(
      Pose2d pose,
      ChassisSpeeds currentSpeeds,
      double maxSpeed,
      boolean useGoal,
      Rotation2d goalRotation) {
    double stepSize = maxSpeed * Constants.PERIOD.in(Seconds);

    if (goalPosition.isEmpty()) {
      return sample(pose.getTranslation(), pose.getRotation(), 0, 0, 0);
    }

    long startTime = System.nanoTime();

    Translation2d goal = goalPosition.get();
    Translation2d position = pose.getTranslation();
    Translation2d error = position.minus(goal);

    if (useGoal && error.getNorm() < stepSize * 1.5) {
      return sample(goal, goalRotation, 0, 0, 0);
    }

    Force netForce =
        getObstacleForce(position, goal)
            .plus(getWallForce(position, goal))
            .plus(useGoal ? getGoalForce(position, goal) : Force.kZero);

    if (useGoal) {
      stepSize = Math.min(maxSpeed, maxSpeed * Math.min(error.getNorm() / 2, 1)) * CONTROL_PERIOD;
    }

    Translation2d step = new Translation2d(stepSize, netForce.getAngle());
    Translation2d nextPosition = position.plus(step);

    long endTime = System.nanoTime();
    Epilogue.getConfig().backend.log("/lib/repulsorTimeS", (endTime - startTime));

    previousSample =
        sample(
            nextPosition,
            goalRotation,
            step.getX() / CONTROL_PERIOD,
            step.getY() / CONTROL_PERIOD,
            0);
    return previousSample;
  }

  /**
   * Generates a trajectory from current position to goal.
   *
   * @param current Current position.
   * @param goal Goal position.
   * @param stepSize Step size in meters.
   * @return List of waypoints forming the trajectory.
   */
  public ArrayList<Translation2d> getTrajectory(
      Translation2d current, Translation2d goal, double stepSize) {
    pathLength = 0;
    ArrayList<Translation2d> trajectory = new ArrayList<>();
    Translation2d position = current;

    for (int i = 0; i < MAX_TRAJECTORY_ITERATIONS; i++) {
      Translation2d error = position.minus(goal);
      if (error.getNorm() < stepSize * 1.5) {
        trajectory.add(goal);
        break;
      }

      Force netForce = getForce(position, goal);
      if (netForce.getNorm() == 0) {
        break;
      }

      Translation2d step = new Translation2d(stepSize, netForce.getAngle());
      Translation2d nextPosition = position.plus(step);
      trajectory.add(nextPosition);
      pathLength += stepSize;
      position = nextPosition;
    }

    return trajectory;
  }
}
