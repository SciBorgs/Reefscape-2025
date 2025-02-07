package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

public class ElevatorConstants {
  public static final double kP = 5;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0.071465;
  public static final double kG = 0.088107;
  public static final double kV = 3.7759;
  public static final double kA = 0.078693;

  public static final Distance POSITION_TOLERANCE = Meters.of(0.005);

  public static final Distance MIN_EXTENSION = Meters.of(0);
  public static final Distance MAX_EXTENSION = Meters.of(1.455);

  public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(2);
  public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(2.8);

  public static final Mass WEIGHT = Pounds.of(6.142);
  public static final double GEARING = 9.375;
  public static final Distance SPROCKET_RADIUS = Inches.of(1.7565 / 2);
  public static final Distance SPROCKET_CIRCUMFRENCE = SPROCKET_RADIUS.times(2 * Math.PI);

  // Input to Output; "values greater than one represent a reduction"
  /** conversion factor in METERS PER ROTATION */
  public static final double CONVERSION_FACTOR = GEARING / SPROCKET_CIRCUMFRENCE.in(Meters) / 2;

  public static final Current CURRENT_LIMIT = Amps.of(50);
}
