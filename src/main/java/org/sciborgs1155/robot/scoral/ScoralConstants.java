package org.sciborgs1155.robot.scoral;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class ScoralConstants {
  public static final double SCORE_POWER = 0.5;
  public static final double L1_POWER = 0.2;

  public static final Current STATOR_LIMIT = Amps.of(35);
  public static final Current CURRENT_LIMIT = Amps.of(30);

  public static final Time RAMP_TIME = Milliseconds.of(50);

  public static final double kS = 0;
  public static final double kV = 0.2;
  public static final double kA = 0.5;
}
