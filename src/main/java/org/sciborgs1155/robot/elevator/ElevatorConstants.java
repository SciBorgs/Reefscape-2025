package org.sciborgs1155.robot.elevator;

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

  public static final Distance MIN_HEIGHT = Meters.of(0);
  public static final Distance MAX_HEIGHT = Meters.of(1);

  public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(1);
  public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(1);

  public static final Mass WEIGHT = Kilograms.of(20);
  public static final Distance DRUM_RADIUS = Meters.of(.1);
  public static final double gearing = 20 / 1;
}
