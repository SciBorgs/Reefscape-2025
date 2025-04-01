package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.sciborgs1155.robot.drive.DriveConstants;

/**
 * Constants is a globally accessible class for storing immutable values. Every value should be
 * <code>public static final</code>.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * @see Units
 */
public class Constants {
  /** Returns the robot's alliance. */
  public static Alliance alliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red);
  }

  /** Returns the rotation of the robot's alliance with respect to the origin. */
  public static Rotation2d allianceRotation() {
    return Rotation2d.fromRotations(alliance() == Alliance.Blue ? 0 : 0.5);
  }

  public static enum RobotType {
    FULL,
    CHASSIS,
    NONE,
    COROLLING,
    SCORALING
  }

  /**
   * Returns the reflection of a pose about the center of the field, effectively swapping alliances.
   *
   * @param pose The pose being reflected.
   * @param allianceDependent If true, doesn't reflect if on the blue alliance. If false, reflects
   *     anyway.
   */
  public static Pose2d allianceReflect(Pose2d pose) {
    return alliance() == Alliance.Blue
        ? pose
        : new Pose2d(
            pose.getTranslation()
                .rotateAround(
                    new Translation2d(FieldConstants.LENGTH.div(2), FieldConstants.WIDTH.div(2)),
                    Rotation2d.k180deg),
            pose.getRotation().plus(Rotation2d.k180deg));
  }

  /**
   * A transform that will translate the pose robot-relative right by a certain distance. Negative
   * distances will move the pose left.
   *
   * @distance The distance that the pose will be moved.
   * @return A transform to strafe a pose.
   */
  public static Transform2d strafe(Distance distance) {
    return new Transform2d(
        new Translation2d(distance.in(Meters), Rotation2d.kCW_90deg), Rotation2d.kZero);
  }

  /**
   * A transform that will translate the pose robot-relative forward by a certain distance. Negative
   * distances will move the pose backward.
   *
   * @distance The distance that the pose will be moved.
   * @return A transform to move a pose forward.
   */
  public static Transform2d advance(Distance distance) {
    return new Transform2d(
        new Translation2d(distance.in(Meters), Rotation2d.kZero), Rotation2d.kZero);
  }

  /**
   * A transform that will rotate the pose.
   *
   * @param rotation The rotation to rotate the pose.
   * @return A transform that rotates a pose by a certain Rotation2d.
   */
  public static Transform2d rotate(Rotation2d rotation) {
    return new Transform2d(Translation2d.kZero, rotation);
  }
  
  /** 
   * Clamps a double between two other doubles; If it was already in between, this does nothing.
   *
   * @param value The value being clamped.
   * @param min The maximum value.
   * @param max The minimum value.
   * @return A new double, in between the min and max.
   */
  public static double clamp(double value, double min, double max) {
    return Math.min(Math.max(min, value), max);
  }

  public static RobotType ROBOT_TYPE = RobotType.SCORALING;

  public static boolean TUNING = true;

  /**
   * Creates a Vector from polar coordinates.
   *
   * @param magnitude The magnitude of the vector.
   * @param direction The direction of the vector.
   * @return A Vector from the given polar coordinates.
   */
  public static Vector<N2> fromPolarCoords(double magnitude, Rotation2d direction) {
    return VecBuilder.fill(magnitude * direction.getCos(), magnitude * direction.getSin());
  }

  /** Describes physical properites of the robot. */
  public static class Robot {
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.2);

    public static final Distance SIDE_LENGTH = Inches.of(28);
    public static final Distance BUMPER_LENGTH = SIDE_LENGTH.plus(Inches.of(3));
  }

  public static final Time PERIOD = Seconds.of(0.02); // roborio tickrate (s)
  public static final Time ODOMETRY_PERIOD = Seconds.of(1.0 / 20.0); // 4 ms (speedy!)
  public static final double DEADBAND = 0.15;
  public static final double MAX_RATE =
      DriveConstants.MAX_ACCEL.baseUnitMagnitude()
          / DriveConstants.MAX_ANGULAR_SPEED.baseUnitMagnitude();
  public static final double SLOW_SPEED_MULTIPLIER = 0.33;
  public static final double FULL_SPEED_MULTIPLIER = 1.0;
  public static final String CANIVORE_NAME = "drivetrain";
}
