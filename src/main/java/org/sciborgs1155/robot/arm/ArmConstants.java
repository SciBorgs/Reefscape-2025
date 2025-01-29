package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

/** Constants for the {@link Arm} subsystem. */
public class ArmConstants {
  public static final double kP = 1;
  public static final double kI = 0;
  public static final double kD = 0.1;

  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kA = 0;
  public static final double kG = 0;

  public static final Angle INTAKE_ANGLE = Radians.of(-Math.PI / 4);
  public static final Angle HP_OUTTAKE_ANGLE = Radians.of(Math.PI * 3 / 4);
  public static final Angle PROCESSOR_OUTTAKE_ANGLE = Radians.of(Math.PI * 3 / 4);
  public static final Angle TROUGH_OUTTAKE_ANGLE = Radians.of(Math.PI * 3 / 4);

  public static final Angle STARTING_ANGLE = Radians.of(Math.PI / 2);
  public static final Angle DEFAULT_ANGLE = Radians.of(Math.PI * 5 / 8);
  public static final Angle CLIMB_INTAKE_ANGLE = Radians.of(0);
  public static final Angle CLIMB_FINAL_ANGLE = Radians.of(Math.PI * 3 / 4);

  public static final DCMotor GEARBOX = DCMotor.getKrakenX60(2);
  public static final double GEARING = 8.21;
  public static final double MOI = 0.001;

  public static final Distance ARM_LENGTH = Centimeters.of(10);

  /** Fully extended. */
  public static final Angle MIN_ANGLE = Radians.of(-Math.PI / 4 - 0.2);

  /** Fully retracted. */
  public static final Angle MAX_ANGLE = Radians.of(Math.PI * 3 / 4 + 0.1);

  public static final Angle POSITION_TOLERANCE = Degrees.of(2);

  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(4);
  public static final AngularAcceleration MAX_ACCEL = RadiansPerSecondPerSecond.of(10);

  public static final Current SUPPLY_LIMIT = Amps.of(60);
  public static final Current CLIMB_LIMIT = Amps.of(120);
  public static final Current STRATOR_LIMIT = Amps.of(60);
}
