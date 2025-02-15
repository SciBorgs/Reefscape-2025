package org.sciborgs1155.robot.led;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class LEDConstants {
  // The length of the LED Strip.
  public static final int LED_LENGTH = 120;
  // The distance between two LEDs on the LED Strip.
  public static final Distance LED_SPACING = Meters.of(0.01);

  public static final LEDSegment LEFT_LED_SEGMENT = new LEDSegment(0, 59, false);
  public static final LEDSegment RIGHT_LED_SEGMENT = new LEDSegment(60, 119, true);
  public static final LEDSegment[] LED_SEGMENTS = {LEFT_LED_SEGMENT, RIGHT_LED_SEGMENT};
}
