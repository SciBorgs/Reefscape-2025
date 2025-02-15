package org.sciborgs1155.robot.led;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.led.LEDConstants.LED_SPACING;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Constants;

public class LEDSegment {
  public final int startLED;
  public final int endLED;
  public final AddressableLEDBuffer buffer;
  public LEDPattern pattern;

  public LEDSegment(int start, int end) {
    startLED = start;
    endLED = end;
    pattern = LEDPattern.kOff;
    buffer = new AddressableLEDBuffer(end - start + 1);
  }

  public boolean inRange(int index) {
    return startLED <= index && index <= endLED;
  }

  /** Rainbow LEDs, scrolling at 0.5 m/s. Very cool. */
  public void rainbow() {
    set(LEDPattern.rainbow(225, 225).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), LED_SPACING));
  }

  /**
   * A gradient of green to yellow LEDs, with an applied mask of how much the elevator is raised.
   *
   * @param percent A double supplier that supplies the elevator's percent raised.
   */
  public void elevatorLED(DoubleSupplier percent) {
    set(
        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kYellow)
            .mask(LEDPattern.progressMaskLayer(percent)));
  }

  /** A gradient of green to yellow LEDs, moving at 60 bpm, which synchronizes with many song. */
  public void music() {
    set(
        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kYellow)
            .mask(
                LEDPattern.progressMaskLayer(
                    () ->
                        Math.sin(RobotController.getMeasureTime().in(Seconds) * Math.PI * 2) / 3
                            + 0.5)));
  }

  /** An alernating pattern of yellow and green that is scrolled through. */
  public void scrolling() {
    set(
        alternatingColor(Color.kYellow, 6, Color.kGreen, 10)
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), LED_SPACING));
  }

  /** A breathing gradient that matches the alliance colors. */
  public void autos() {
    if (Constants.alliance() == Alliance.Red) {
      set(
          LEDPattern.gradient(
                  LEDPattern.GradientType.kDiscontinuous, Color.kFirstRed, Color.kOrangeRed)
              .breathe(Seconds.of(2)));
    } else {
      set(
          LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kFirstBlue, Color.kAqua)
              .breathe(Seconds.of(2)));
    }
  }

  public void set(LEDPattern newPattern) {
    pattern = newPattern;
  }

  public void update() {
    pattern.applyTo(buffer);
  }

  private static LEDPattern alternatingColor(
      Color color1, int color1length, Color color2, int color2length) {
    return (reader, writer) -> {
      int bufLen = reader.getLength();
      for (int i = 0; i < bufLen; i++) {
        writer.setLED(i, (((i % (color1length + color2length)) < color1length) ? color1 : color2));
      }
    };
  }
}
