package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import java.util.List;

/**
 * Constants for our 2025 Swerve X2t drivetrain. All fields in this file should be updated for the
 * current robot configuration!
 */
public final class DriveConstants {
  /** The type of control loop to use when controlling a module's drive motor. */
  public static enum ControlMode {
    CLOSED_LOOP_VELOCITY,
    OPEN_LOOP_VELOCITY;
  }

  // The control loop used by all of the modules when driving
  public static final ControlMode DRIVE_MODE = ControlMode.OPEN_LOOP_VELOCITY;

  // Rate at which sensors update periodicially
  public static final Time SENSOR_PERIOD = Seconds.of(0.02);

  // Distance between centers of right and left wheels on robot
  public static final Distance TRACK_WIDTH = Meters.of(0.5931); // .5884 //.5931
  // Distance between front and back wheels on robot
  public static final Distance WHEEL_BASE = Meters.of(0.59077); // .59077
  // The radius of any swerve wheel
  public static final Distance WHEEL_RADIUS = Inches.of(2); // 4/00 dia, 2.00 rad
  // Distance from the center to any wheel of the robot
  public static final Distance RADIUS = TRACK_WIDTH.div(2).times(Math.sqrt(2));
  // Coefficient of friction between the drive wheel and the carpet.
  public static final double WHEEL_COF = 1.0;
  // Robot width with bumpers
  public static final Distance CHASSIS_WIDTH = Inches.of(32.645);

  // Maximum achievable translational and rotation velocities and accelerations of the robot.
  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(5.74);
  public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(16.0);
  public static final AngularVelocity MAX_ANGULAR_SPEED =
      RadiansPerSecond.of(MAX_SPEED.in(MetersPerSecond) / RADIUS.in(Meters));
  public static final AngularAcceleration MAX_ANGULAR_ACCEL =
      RadiansPerSecond.per(Second).of(MAX_ACCEL.in(MetersPerSecondPerSecond) / RADIUS.in(Meters));

  // Arbitrary max rotational velocity for the driver to effectively control the robot
  public static final AngularVelocity TELEOP_ANGULAR_SPEED = Radians.per(Second).of(2 * Math.PI);

  public static final Translation2d[] MODULE_OFFSET = {
    new Translation2d(WHEEL_BASE.div(2), TRACK_WIDTH.div(2)), // front left
    new Translation2d(WHEEL_BASE.div(2), TRACK_WIDTH.div(-2)), // front right
    new Translation2d(WHEEL_BASE.div(-2), TRACK_WIDTH.div(2)), // rear left
    new Translation2d(WHEEL_BASE.div(-2), TRACK_WIDTH.div(-2)) // rear right
  };

  // angular offsets of the modules, since we use absolute encoders
  // ignored (used as 0) in simulation because the simulated robot doesn't have offsets
  public static final List<Rotation2d> ANGULAR_OFFSETS =
      List.of(
          Rotation2d.fromRadians(0), // front left
          Rotation2d.fromRadians(0), // front right
          Rotation2d.fromRadians(0), // rear left
          Rotation2d.fromRadians(0) // rear right
          );

  public static final Rotation3d GYRO_OFFSET = new Rotation3d(0, 0, Math.PI);

  // TODO: Change ALL characterization constants for each unique robot as needed.
  public static final class Translation {
    public static final double P = 3.0;
    public static final double I = 0.0;
    public static final double D = 0.05;

    public static final Distance TOLERANCE = Centimeters.of(5);
  }

  public static final class Rotation {
    public static final double P = 4.5;
    public static final double I = 0.0;
    public static final double D = 0.05;

    public static final Angle TOLERANCE = Degrees.of(3);
  }

  public static final class ModuleConstants {
    public static final double COUPLING_RATIO = 0;

    public static final class Driving {
      public static final Distance CIRCUMFERENCE = Inches.of(4.0 * Math.PI);

      public static final double GEARING = 5.68;

      public static final Current CURRENT_LIMIT = Amps.of(60);

      public static final class PID {
        public static final double P = 3.2;
        public static final double I = 0.0;
        public static final double D = 0.0;
      }

      public static final class FF {
        public static final double S = 0.088468;
        public static final double V = 2.1314;
        public static final double A = 0.33291;
      }
    }

    static final class Turning {
      public static final double GEARING = 12.1;
      public static final double CANCODER_GEARING = 1; // 0.4 / (12.1 * 1.3);

      public static final Current CURRENT_LIMIT = Amps.of(20);

      public static final class PID {
        public static final double P = 14;
        public static final double I = 0.0;
        public static final double D = 0.05;
      }

      // system constants only used in simulation
      public static final class FF {
        public static final double S = 0.30817;
        public static final double V = 0.55;
        public static final double A = 0.03;
      }
    }
  }
}
