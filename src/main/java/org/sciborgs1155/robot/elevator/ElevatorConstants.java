package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

public class ElevatorConstants {
  public static final double kP = 3;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kS = 0.0755;
  public static final double kG = 0.2645;
  public static final double kV = 3.7;
  public static final double kA = 0.012;

  public static final Distance POSITION_TOLERANCE = Centimeters.of(3.5); // (1.0);
  public static final double VELOCITY_TOLERANCE = 0.005; // (0.1);

  public static final Distance MIN_EXTENSION = Meters.of(0.0);
  public static final Distance MAX_EXTENSION = Meters.of(1.455);

  public static final Translation3d BASE_FROM_CHASSIS = new Translation3d(-0.04, -0.2, .125);
  public static final Translation3d CARRIAGE_FROM_CHASSIS = new Translation3d(.015, -0.2, .1);

  public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(2);
  public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(2.8);

  public static final Mass WEIGHT = Pounds.of(6.142);
  public static final double GEARING = 9.375;
  public static final Distance SPROCKET_RADIUS = Inches.of(1.7565 / 2);
  public static final Distance SPROCKET_CIRCUMFRENCE = SPROCKET_RADIUS.times(2 * Math.PI);

  // Input to Output; "values greater than one represent a reduction"
  /** conversion factor in METERS PER ROTATION */
  public static final double CONVERSION_FACTOR = GEARING / SPROCKET_CIRCUMFRENCE.in(Meters) / 2;

  public static final Current CURRENT_LIMIT = Amps.of(65);

  // Don't worry about this :)
  public static final Time HIGH_FIVE_DELAY = Seconds.of(.3);
  public static final Distance RAY_HIGH = Meter.of(1.086);
  public static final Distance RAY_MIDDLE = Meter.of(0.628);
  public static final Distance RAY_LOW = Meter.of(0.25);

  public enum Level {
    L1(Meters.of(0.3)),
    L2(Meters.of(0.488)),
    L3(Meters.of(0.838)),
    L4(Meters.of(1.408)),

    L2_ALGAE(Meters.of(0.119)),
    L3_ALGAE(Meters.of(0.496));

    public final Distance extension;

    Level(Distance extension) {
      this.extension = extension;
    }
  }
}
