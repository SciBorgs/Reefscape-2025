package org.sciborgs1155.robot.scoral;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class ScoralConstants {
  public static final double POWER = 0.5;
  public static final Current CURRENT_LIMI = Amps.of(45);
  public static final Time RAMP_TIME = Milliseconds.of(50);
}
