package org.sciborgs1155.robot.led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

public class LedConstants {
  // Lengt6h of the LED Strip.
  public static final int LED_LENGTH = 64;
  public static final Distance LED_SPACING = Meters.of(0.01);

  public static final LEDPattern RAINBOW =
      LEDPattern.rainbow(225, 225).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), LED_SPACING);
  public static final LEDPattern MUSIC_60_BPM =
      LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kYellow)
          .mask(
              LEDPattern.progressMaskLayer(
                  () ->
                      Math.sin(RobotController.getMeasureTime().in(Seconds) * Math.PI * 2) / 3
                          + 0.5));
}
