package org.sciborgs1155.robot.led;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Ports.LEDs.*;
import static org.sciborgs1155.robot.led.LEDConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.BooleanSupplier;
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

  /**
   * Represents a set of LEDs on the full LED strip, which allows for different patterns to run
   * simultaneously on different regions of the LED strip.
   *
   * @param start The starting LED index, inclusive.
   * @param end The ending LED index, inclusive.
   * @param invert Whether or not apply the pattern backwards.
   */
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
    setDefaultCommand(
        run(
            () ->
                update(
                    LEDPattern.rainbow(225, 225)
                        .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), LED_SPACING))));
  }

  /** Rainbow LEDs, scrolling at 0.5 m/s. Very cool. */
  public Command solid(Color color) {
    return set(LEDPattern.solid(color));
  }

  /** Rainbow LEDs, scrolling at 0.5 m/s. Very cool. */
  public Command rainbow() {
    return set(
        LEDPattern.rainbow(225, 225).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), LED_SPACING));
  }

  /**
   * Sets a percent of the strip to a color from red to green LEDs, both depending on a given
   * percent.
   *
   * @param percent A double supplier that represents percent completion.
   */
  public Command progressGradient(DoubleSupplier percent) {
    return set(
        solidGradient(percent).mask(LEDPattern.progressMaskLayer(() -> 1 - percent.getAsDouble())));
  }

  /**
   * A gradient of red to green LEDs representing how much the elevator is raised, until the
   * elevator reaches its setpoint, where it then solid lime.
   *
   * @param percent A double supplier that supplies the elevator's percent raised.
   * @param atGoal A boolean supplier that supplies whether the elevator is at its goal.
   */
  public Command progressGradient(DoubleSupplier percent, BooleanSupplier atGoal) {
    return set(solidGradient(percent)
            .mask(LEDPattern.progressMaskLayer(() -> 1 - percent.getAsDouble())))
        .until(atGoal)
        .andThen(solid(Color.kAqua));
  }

  /**
   * Sets the LEDPattern based on an error. When the error is within tolerance, LEDs blink blue.
   *
   * @param percentError The error. 1 is a really bad error while 0 is no error.
   * @param tolerance The allowed tolerance in error.
   */
  public Command error(DoubleSupplier error, double tolerance) {
    return set(solidGradient(error))
        .until(() -> error.getAsDouble() < tolerance)
        .andThen(blink(Color.kAqua));
  }

  /** A gradient of green to yellow LEDs, moving at 60 bpm, which synchronizes with many song. */
  public Command music() {
    return set(
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kYellow)
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
                  LEDPattern.GradientType.kContinuous, Color.kFirstRed, Color.kOrangeRed)
              .breathe(Seconds.of(1)));
    } else {
      return set(
          LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kFirstBlue, Color.kAqua)
              .breathe(Seconds.of(1)));
    }
  }

  /** Blinks the LEDStrip white for 0.15 seconds, with a followng 0.15 seconds rest, twice. */
  public Command blink(Color color) {
    return set(LEDPattern.solid(color).blink(Seconds.of(0.2)));
  }

  /** Applies an LEDPattern to the set of LEDs controlled by the LEDStrip. */
  public Command set(LEDPattern pattern) {
    return run(() -> update(pattern)).asProxy();
  }

  private void update(LEDPattern pattern) {
    (inverted ? pattern.reversed() : pattern).applyTo(selfBuffer);
    for (int i = startLED; i <= endLED; i++) {
      allBuffer.setLED(i, selfBuffer.getLED(i - startLED));
    }
    led.setData(allBuffer);
  }

  /** Alternates between two colors, for a given length for each. */
  private static LEDPattern alternatingColor(
      Color color1, int color1length, Color color2, int color2length) {
    return (reader, writer) -> {
      int bufLen = reader.getLength();
      for (int i = 0; i < bufLen; i++) {
        writer.setLED(i, (((i % (color1length + color2length)) < color1length) ? color1 : color2));
      }
    };
  }

  /** Sets a solid color of a range from red to green, given an error double supplier. */
  private static LEDPattern solidGradient(DoubleSupplier error) {
    return (reader, writer) -> {
      Color color =
          Color.fromHSV(
              (int) MathUtil.clamp(Math.round((1 - error.getAsDouble()) * 50), 0, 50), 255, 255);
      int bufLen = reader.getLength();
      for (int i = 0; i < bufLen; i++) {
        writer.setLED(i, color);
      }
    };
  }

  @Override
  public void periodic() {
    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }

  @Override
  public void close() throws Exception {
    led.close();
  }
}
