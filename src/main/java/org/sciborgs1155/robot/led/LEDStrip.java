package org.sciborgs1155.robot.led;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Ports.LEDs.*;
import static org.sciborgs1155.robot.led.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Logged;
import org.sciborgs1155.robot.Constants;

public class LEDStrip extends SubsystemBase implements Logged, AutoCloseable {
  private static final AddressableLED led = new AddressableLED(LED_PORT);
  private static final AddressableLEDBuffer allBuffer = new AddressableLEDBuffer(LED_LENGTH);
  private static boolean ledInitalized = false;

  public final int startLED;
  public final int endLED;
  public final boolean inverted;
  private final AddressableLEDBuffer selfBuffer;
  public LEDPattern pattern;

  public LEDStrip(int start, int end, boolean invert) {
    startLED = start;
    endLED = end;
    inverted = invert;
    if (!ledInitalized) {
      ledInitalized = true;
      led.setLength(LED_LENGTH);
      led.setData(allBuffer);
      led.start();
    }
    selfBuffer = new AddressableLEDBuffer(end - start + 1);
  }

  /** Rainbow LEDs, scrolling at 0.5 m/s. Very cool. */
  public Command rainbow() {
    return set(
        LEDPattern.rainbow(225, 225).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), LED_SPACING));
  }

  /**
   * A gradient of green to yellow LEDs, with an applied mask of how much the elevator is raised.
   *
   * @param percent A double supplier that supplies the elevator's percent raised.
   */
  public Command elevatorLED(DoubleSupplier percent) {
    return set(
        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kYellow)
            .mask(LEDPattern.progressMaskLayer(percent)));
  }

  /** A gradient of green to yellow LEDs, moving at 60 bpm, which synchronizes with many song. */
  public Command music() {
    return set(
        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kYellow)
            .mask(
                LEDPattern.progressMaskLayer(
                    () ->
                        Math.sin(RobotController.getMeasureTime().in(Seconds) * Math.PI * 2) / 3
                            + 0.5)));
  }

  /** An alernating pattern of yellow and green that is scrolled through. */
  public Command scrolling() {
    return set(
        alternatingColor(Color.kYellow, 6, Color.kGreen, 10)
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), LED_SPACING));
  }

  /** A breathing gradient that matches the alliance colors. */
  public Command autos() {
    if (Constants.alliance() == Alliance.Red) {
      return set(
          LEDPattern.gradient(
                  LEDPattern.GradientType.kDiscontinuous, Color.kFirstRed, Color.kOrangeRed)
              .breathe(Seconds.of(2)));
    } else {
      return set(
          LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kFirstBlue, Color.kAqua)
              .breathe(Seconds.of(2)));
    }
  }

  /** Blinks the LEDStrip white for 0.15 seconds, with a followng 0.15 seconds rest, twice. */
  public Command blink() {
    return set(LEDPattern.solid(Color.kWhite).blink(Seconds.of(0.15))).withTimeout(0.6);
  }

  public Command set(LEDPattern pattern) {
    return run(
        () -> {
          (inverted ? pattern.reversed() : pattern).applyTo(selfBuffer);
          for (int i = startLED; i <= endLED; i++) {
            allBuffer.setLED(i, selfBuffer.getLED(i - startLED));
          }
          led.setData(allBuffer);
        });
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

  @Override
  public void close() throws Exception {
    led.close();
  }
}
