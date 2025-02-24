package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
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
  // TODO: Modify as needed.
  /** Returns the robot's alliance. */
  public static Alliance alliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  /** Returns the rotation of the robot's alliance with respect to the origin. */
  public static Rotation2d allianceRotation() {
    return Rotation2d.fromRotations(alliance() == Alliance.Blue ? 0 : 0.5);
  }

  /** Describes physical properites of the robot. */
  public static class Robot {
    public static final Mass MASS = Kilograms.of(25);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.2);
    public static final Distance BUMPER_LENGTH = Inches.of(34);
    public static final Distance TRACK_LENGTH = Inches.of(23);
  }

  // maple
  public static final Voltage DFV = Volts.of(10); // Drive friction voltage
  public static final Voltage SFV = Volts.of(10); // Steer friction voltage
  public static final Distance WR = Inches.of(12); // Wheel Radius
  public static final double CWF = 0.1; // Wheel coefficient of friction
  public static final MomentOfInertia SMOI =
      KilogramSquareMeters.of(0.2); // Steer moment of inertia
  public static final double DGR = 12.2; // Drive Gear Ratio
  public static final double SGR = 5.2; // Steer Gear Ratio

  public static final Time PERIOD = Seconds.of(0.02); // roborio tickrate (s)
  public static final double DEADBAND = 0.15;
  public static final double MAX_RATE =
      DriveConstants.MAX_ACCEL.baseUnitMagnitude()
          / DriveConstants.MAX_ANGULAR_SPEED.baseUnitMagnitude();
  public static final double SLOW_SPEED_MULTIPLIER = 0.33;
  public static final double FULL_SPEED_MULTIPLIER = 1.0;
  public static final String CANIVORE_NAME = "drivetrain";

  // Origin at corner of blue alliance side of field
  public static class Field {
    public static final Distance LENGTH = Centimeters.of(1755);
    public static final Distance WIDTH = Centimeters.of(805);

    /** Returns whether the provided position is within the boundaries of the field. */
    public static boolean inField(Pose3d pose) {
      return (pose.getX() > 0
          && pose.getX() < Field.LENGTH.in(Meters)
          && pose.getY() > 0
          && pose.getY() < Field.WIDTH.in(Meters));
    }
  }

  public static final DriveTrainSimulationConfig SIM_DRIVE_CONFIG =
      DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getKrakenX60(1),
                  5.68, // Drive Motor Gear Ratio
                  12.2, // Steer Motor Gear Ratio
                  Volts.of(0.1), // Drive Friction Voltage
                  Volts.of(0.1), // Steer Friction Voltage
                  Inches.of(2), // Wheel Radius
                  KilogramSquareMeters.of(0.2), // Steer Moment of Inertia
                  1.2 // Wheel Coefficient of Friction
                  ))
          .withTrackLengthTrackWidth(Robot.TRACK_LENGTH, Robot.TRACK_LENGTH)
          .withBumperSize(Robot.BUMPER_LENGTH, Robot.BUMPER_LENGTH);
}
