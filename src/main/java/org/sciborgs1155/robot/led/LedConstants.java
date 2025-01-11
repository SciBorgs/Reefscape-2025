package org.sciborgs1155.robot.led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;

public class LedConstants {
  // Lengt6h of the LED Strip.
  public static final int LED_LENGTH = 64;
  public static final Distance LED_SPACING = Meters.of(0.01);

  public static final LEDPattern RAINBOW =
      LEDPattern.rainbow(200, 200).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.25), LED_SPACING);
}
