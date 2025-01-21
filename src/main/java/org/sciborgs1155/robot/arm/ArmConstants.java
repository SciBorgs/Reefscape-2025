package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

/** Constants for the {@link Arm} subsystem. */
public class ArmConstants {
  public static final Angle INTAKE_ANGLE = Radians.of(-Math.PI / 4);
  public static final Angle PROCESSOR_OUTTAKE_ANGLE = Radians.of(Math.PI * 3 / 4);
  public static final Angle TROUGH_OUTTAKE_ANGLE = Radians.of(Math.PI * 3 / 4);

  public static final Angle STARTING_ANGLE = Radians.of(Math.PI / 2);
  public static final Angle DEFAULT_ANGLE = Radians.of(Math.PI * 5 / 8);
  public static final Angle CLIMB_INTAKE_ANGLE = Radians.of(0);
  public static final Angle CLIMB_FINAL_ANGLE = Radians.of(Math.PI * 3 / 4);

  /** Fully extended. */
  public static final Angle MIN_ANGLE = Radians.of(-Math.PI / 4 - 0.2);

  /** Fully retracted. */
  public static final Angle MAX_ANGLE = Radians.of(Math.PI * 3 / 4 + 0.1);
}
