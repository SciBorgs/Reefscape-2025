package org.sciborgs1155.robot.scoral;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class ScoralConstants {
  public static final double SCORE_POWER = 0.85;
  public static final double INTAKE_POWER = 0.4;

  public static final Current STATOR_LIMIT = Amps.of(55);
  public static final Current CURRENT_LIMIT = Amps.of(50);

  public static final Time RAMP_TIME = Milliseconds.of(50);
}
