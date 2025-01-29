package org.sciborgs1155.robot.coroller;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;

/** Constants for the {@link Coroller} subsystem. */
public class CorollerConstants {
  /** Power from -1 to 1 (negative values indicate reversal of direction). */
  public static final double INTAKE_POWER = 0.5;

  /** Power from -1 to 1 (negative values indicate reversal of direction). */
  public static final double OUTTAKE_POWER = -0.2;

  public static final Current STRATOR_LIMIT = Amps.of(60);
  public static final Current SUPPLY_LIMIT = Amps.of(60);
}
