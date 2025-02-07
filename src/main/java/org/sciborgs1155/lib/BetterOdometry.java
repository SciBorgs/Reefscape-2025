package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.function.Supplier;

public class BetterOdometry {

  private Translation2d fieldPosition;
  private SwerveModuleState[] prevStates;
  private final Supplier<SwerveModuleState[]> modules;
  private final Supplier<Rotation2d> heading;

  /**
   * Creates a new arc odometry calculator.
   *
   * @param modules The supplier of the modules' states.
   * @param heading The supplier for the field-relative heading of the robot.
   * @param start The starting field-relative position for the robot.
   */
  public BetterOdometry(
      Supplier<SwerveModuleState[]> modules, Supplier<Rotation2d> heading, Translation2d start) {
    prevStates = modules.get(); // to prevent a null pointer exception

    fieldPosition = start;
    this.modules = modules;
    this.heading = heading;
  }

  /**
   * Resets the field position of the odometry to a given position.
   *
   * @param pos The field-relative position to reset to.
   */
  public void resetOdometry(Pose2d pose) {
    fieldPosition = pose.getTranslation();
  }

  /**
   * Uses two tangential velocities of an arc to find the secant length of one module, representing
   * the module's diplacement.
   *
   * @param v0 The starting velocity vector of the module, (at t0).
   * @param v1 The ending velocity vector of the module (at t1).
   * @return The calculated translation vector from where it is at t0 to where it is at t1.
   */
  public static Translation2d moduleDisplacement(Translation2d v0, Translation2d v1) {
    // Angle between the two velocities

    // in case either is the zero vector (getting the angle wouldn't work)
    if (v0.getNorm() == 0 || v1.getNorm() == 0) {
      return (v1.plus(v0)).times(0.5).times(PERIOD.in(Seconds));
    }

    Rotation2d theta = v1.getAngle().minus(v0.getAngle());

    if (theta.getRadians() == 0) {
      // just making sure we dont divide by 0
      return ((v0.plus(v1)).div(2)).times(PERIOD.in(Seconds));
    }

    // Secant length formula, then rotate it by the average angle between the two velocities
    return new Translation2d(
        (PERIOD.in(Seconds) * (v0.plus(v1).getNorm()) / theta.getRadians())
            * Math.sin(theta.getRadians() / 2),
        (v0.getAngle().plus(v1.getAngle())).div(2));
  }

  /**
   * Takes the robot-relative velocity of the module.
   *
   * @param state The input module state.
   * @return The robot-relative velocity of the module.
   */
  private static Translation2d stateToVelocity(SwerveModuleState state) {
    return new Translation2d(state.speedMetersPerSecond, state.angle);
  }

  /**
   * Averages the displacements of all of the motors to find the robot relative displacement.
   *
   * @return The robot relative displacement.
   */
  public Translation2d robotRelativeDisplacement() {
    Translation2d avg = new Translation2d(0, 0);
    for (int i = 0; i < 4; i++) {
      avg =
          avg.plus(
              moduleDisplacement(stateToVelocity(prevStates[i]), stateToVelocity(modules.get()[i]))
                  .div(4));
    } // im so sad i couldnt use a stream here
    return avg;
  }

  /**
   * Rotates the robot relative displacement by the robot's heading, then adds it to the robot's
   * position on the field.
   *
   * @return The robot's field relative position.
   */
  public Translation2d fieldRelativePosition() {
    return fieldPosition.plus(robotRelativeDisplacement().rotateBy(heading.get()));
  }

  /**
   * Updates the position of the odometry using the given suppliers. Should be called periodically.
   */
  public void update() {
    fieldPosition = fieldRelativePosition();
    prevStates = modules.get();
  }

  /**
   * @return The estimated position of the robot.
   */
  public Pose2d getPose() {
    return new Pose2d(fieldPosition, heading.get());
  }
}
