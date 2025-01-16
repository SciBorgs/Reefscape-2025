package org.sciborgs1155.robot.led;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Ports.LEDs.*;
import static org.sciborgs1155.robot.led.LEDConstants_a.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import java.util.function.DoubleSupplier;
import monologue.Logged;
import org.sciborgs1155.robot.Constants;

public class LEDStrip_a extends SubsystemBase implements Logged, AutoCloseable {

  private final AddressableLED led = new AddressableLED(LED_PORT);
  private final AddressableLEDBuffer buffer;

  public LEDStrip_a() {
    led.setLength(LED_LENGTH);
    buffer = new AddressableLEDBuffer(LED_LENGTH);
    led.setData(buffer);
    led.start();
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

  /** A solid yellow green that is scrolled through. */
  public Command scrolling() {
    return set(
        LEDPattern.solid(Color.kGreenYellow)
            .mask(
                LEDPattern.steps(Map.of(0, Color.kWhite, 0.5, Color.kBlack))
                    .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), LED_SPACING)));
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

  public Command set(LEDPattern pattern) {
    return run(
        () -> {
          pattern.applyTo(buffer);
          led.setData(buffer);
        });
  }

  @Override
  public void close() throws Exception {
    led.close();
  }
}
