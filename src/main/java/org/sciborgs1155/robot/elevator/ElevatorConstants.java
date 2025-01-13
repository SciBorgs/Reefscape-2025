package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

public class ElevatorConstants {
  public static final double kP = 10;
  public static final double kI = 0;
  public static final double kD = 10;
  public static final double kS = .1;
  public static final double kG = 1;
  public static final double kV = 1;
  public static final double kA = 0;

  public static final Distance MIN_HEIGHT = Meters.of(.2);
  public static final Distance MAX_HEIGHT = Meters.of(2);

  public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(1);
  public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(1);

  public static final Mass WEIGHT = Kilograms.of(20);
  public static final Distance DRUM_RADIUS = Meters.of(.1);
  public static final double GEARING = 20 / 1;
  public static final Distance SPROCKET_RADIUS = Inches.of(1);
  public static final Distance SPROCKET_CIRCUMFRENCE = SPROCKET_RADIUS.times(2 * Math.PI);
  public static final double CONVERSION_FACTOR = SPROCKET_CIRCUMFRENCE.in(Meters) * 2 / GEARING;

  public static final Distance TEST_HEIGHT = MIN_HEIGHT.plus(Meters.of(.2));
}
