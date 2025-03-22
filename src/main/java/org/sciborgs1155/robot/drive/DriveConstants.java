package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.Robot.BUMPER_LENGTH;
import static org.sciborgs1155.robot.Constants.Robot.MASS;
import static org.sciborgs1155.robot.Constants.Robot.MOI;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import java.util.List;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

/** Constants for our 2025 Swerve X2t drivetrain. */
public final class DriveConstants {
  /** The type of control loop to use when controlling a module's drive motor. */
  public static enum ControlMode {
    CLOSED_LOOP_VELOCITY,
    OPEN_LOOP_VELOCITY;
  }

  // The angle between the velocity and the displacement from a target, above which the robot will
  // not use assisted driving to the target. (the driver must be driving in the general direction of
  // the assisted driving target.)
  public static final Angle ASSISTED_DRIVING_THRESHOLD = Radians.of(Math.PI / 6);

  // The input of the joystick beyond which the assisted driving will not control the rotation of
  // the swerve.
  public static final double ASSISTED_ROTATING_THRESHOLD = 0.02;

  // The control loop used by all of the modules when driving
  public static final ControlMode DRIVE_MODE = ControlMode.CLOSED_LOOP_VELOCITY;

  // Rate at which sensors update periodicially
  public static final Time SENSOR_PERIOD = Seconds.of(0.02);

  // Distance between centers of right and left wheels on robot
  public static final Distance TRACK_WIDTH = Meters.of(0.5931);
  // Distance between front and back wheels on robot
  public static final Distance WHEEL_BASE = Meters.of(0.59077);
  // The radius of any swerve wheel
  public static final Distance WHEEL_RADIUS = Inches.of(2);
  // Distance from the center to any wheel of the robot
  public static final Distance RADIUS = TRACK_WIDTH.div(2).times(Math.sqrt(2));
  // Coefficient of friction between the drive wheel and the carpet
  public static final double WHEEL_COF = 1.0;
  // Robot width with bumpers
  public static final Distance CHASSIS_WIDTH = Inches.of(32.645);
  // Robot starting position in sim
  public static final Pose2d SIM_STARTING_POSE =
      new Pose2d(9.68, 3.03, new Rotation2d(Degrees.of(32.54)));

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

  /** Simulation constants that are used for the creation of the maplesim drivetrain. */
  public static final DriveTrainSimulationConfig SIM_DRIVE_CONFIG =
      DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getKrakenX60(1),
                  Driving.GEARING, // Drive Motor Gear Ratio
                  Turning.GEARING, // Steer Motor Gear Ratio
                  Volts.of(Driving.FF.S), // Drive Friction Voltage
                  Volts.of(Turning.FF.S), // Steer Friction Voltage
                  WHEEL_RADIUS, // Wheel Radius
                  KilogramSquareMeters.of(0.02), // Steer Moment of Inertia
                  WHEEL_COF // Wheel Coefficient of Friction
                  ))
          .withTrackLengthTrackWidth(TRACK_WIDTH, TRACK_WIDTH)
          .withBumperSize(BUMPER_LENGTH, BUMPER_LENGTH);

  /**
   * Simulates a drivetrain modeled on real life. In sim, this pose should be treated as the actual
   * robot pose, that the odometry should attempt to model.
   */
  public static final SwerveDriveSimulation driveSim =
      new SwerveDriveSimulation(SIM_DRIVE_CONFIG, SIM_STARTING_POSE);

  public static final RobotConfig ROBOT_CONFIG =
      new RobotConfig(
          MASS,
          MOI,
          new ModuleConfig(
              WHEEL_RADIUS,
              MAX_SPEED,
              WHEEL_COF,
              DCMotor.getKrakenX60(1),
              1 / ModuleConstants.Driving.GEARING,
              ModuleConstants.Driving.STATOR_LIMIT,
              1),
          MODULE_OFFSET);

  public static final PathConstraints PATH_CONSTRAINTS =
      new PathConstraints(MAX_SPEED, MAX_ACCEL, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCEL);

  // How many ticks before it pathfinds again.
  public static final int PATHFINDING_PERIOD = 1;

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

  public static final class Translation {
    public static final double P = 3.0;
    public static final double I = 0.0;
    public static final double D = 0.05;

    public static final Distance TOLERANCE = Centimeters.of(5);

    public static final double PRECISION = 5;
  }

  public static final class Rotation {
    public static final double P = 4.5;
    public static final double I = 0.0;
    public static final double D = 0.05;

    public static final Angle TOLERANCE = Degrees.of(3);

    public static final double PRECISION = 3;
  }

  public static final class ModuleConstants {
    public static final double COUPLING_RATIO = 0;

    public static final class Driving {
      public static final Distance CIRCUMFERENCE = Inches.of(4.0 * Math.PI);

      public static final double GEARING = 5.68;

      public static final Current STATOR_LIMIT = Amps.of(80); // 120A max slip current
      public static final Current SUPPLY_LIMIT = Amps.of(70);

      public static final class PID {
        public static final double P = 3.2;
        public static final double I = 0.0;
        public static final double D = 0.0;
      }

      public static final class FF {
        public static final double S = 0.022436;
        public static final double V = 2.1154;
        public static final double A = 0.45287;
      }
    }

    static final class Turning {
      public static final double GEARING = 12.1;
      public static final double CANCODER_GEARING = 1;

      public static final Current CURRENT_LIMIT = Amps.of(20);

      public static final class PID {
        public static final double P = 35;
        public static final double I = 0.0;
        public static final double D = 0.0;
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
