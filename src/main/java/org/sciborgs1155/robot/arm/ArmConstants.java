package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Constants for the {@link Arm} subsystem. */
public class ArmConstants {
  // TODO: not tuned (at all)
  public static final double kP = 5;
  public static final double kI = 0;
  public static final double kD = 0.01;

  public static final double kS = 1;
  public static final double kV = 0;
  public static final double kA = 0.01;
  public static final double kG = 0.01;

  // TODO: figure these out
  public static final Rotation2d INTAKE_ANGLE = Rotation2d.fromRadians(0);
  public static final Rotation2d OUTTAKE_ANGLE = Rotation2d.fromRadians(0);
  public static final Rotation2d STARTING_ANGLE = Rotation2d.fromRadians(0);

  // TODO: and these...
  public static final DCMotor GEARBOX = DCMotor.getNEO(1);
  public static final double GEARING = 8.21;
  public static final double MOI = 1;

  public static final Distance ARM_LENGTH = Centimeters.of(10);

  /** Fully extended. */
  public static final Angle MIN_ANGLE = Radians.of(-3);

  /** Fully retracted. */
  public static final Angle MAX_ANGLE = Radians.of(3);
}
