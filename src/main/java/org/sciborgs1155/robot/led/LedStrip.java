package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Ports.LEDs.*;
import static org.sciborgs1155.robot.led.LedConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Logged;

public class LedStrip extends SubsystemBase implements Logged, AutoCloseable {

  private final AddressableLED led = new AddressableLED(LED_PORT);
  private final AddressableLEDBuffer buffer;

  public LedStrip() {
    led.setLength(LED_LENGTH);
    buffer = new AddressableLEDBuffer(LED_LENGTH);
    led.setData(buffer);
    led.start();
  }

  /** Rainbow LEDs. Very cool. */
  public Command rainbow() {
    return set(RAINBOW);
  }

  /**
   * A gradient of green to yellow LEDs, with an applied mask of how much the elevator is raised.
   *
   * @param percent A double supplier that supplies the elevator's percent raised.
   */
  public Command elevator(DoubleSupplier percent) {
    return set(
        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kYellow)
            .mask(LEDPattern.progressMaskLayer(percent)));
  }

  /**
   * A gradient of green to yellow LEDs, moving at 60 bpm, which synchronizes with many songs, such
   * as common 120 bpm songs today.
   */
  public Command music() {
    return set(MUSIC_60_BPM);
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
