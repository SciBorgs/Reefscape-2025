package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Ports.LEDs.*;
import static org.sciborgs1155.robot.led.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Logged;

public class LEDStrip extends SubsystemBase implements Logged, AutoCloseable {

  private final AddressableLED led = new AddressableLED(LED_PORT);
  private final AddressableLEDBuffer buffer;

  public LEDStrip() {
    led.setLength(LED_LENGTH);
    buffer = new AddressableLEDBuffer(LED_LENGTH);
    led.setData(buffer);
    led.start();
  }

  /** Rainbow LEDs, scrolling at 0.5 m/s. Very cool. */
  public Command rainbow() {
    return runOnce(
        () -> {
          LEFT_LED_SEGMENT.rainbow();
          RIGHT_LED_SEGMENT.rainbow();
        });
  }

  /**
   * A gradient of green to yellow LEDs, with an applied mask of how much the elevator is raised.
   *
   * @param percent A double supplier that supplies the elevator's percent raised.
   */
  public Command elevatorLED(DoubleSupplier doubleSupplier) {
    return runOnce(
        () -> {
          LEFT_LED_SEGMENT.elevatorLED(doubleSupplier);
          RIGHT_LED_SEGMENT.elevatorLED(doubleSupplier);
        });
  }

  /** A gradient of green to yellow LEDs, moving at 60 bpm, which synchronizes with many song. */
  public Command music() {
    return runOnce(
        () -> {
          LEFT_LED_SEGMENT.music();
          RIGHT_LED_SEGMENT.music();
        });
  }

  public Command blindLeft() {
    return runOnce(
        () -> {
          LEFT_LED_SEGMENT.rainbow();
        });
  }

  /** An alernating pattern of yellow and green that is scrolled through. */
  public Command scrolling() {
    return runOnce(
        () -> {
          LEFT_LED_SEGMENT.scrolling();
          RIGHT_LED_SEGMENT.scrolling();
        });
  }

  /** A breathing gradient that matches the alliance colors. */
  public Command autos() {
    return runOnce(
        () -> {
          LEFT_LED_SEGMENT.autos();
          RIGHT_LED_SEGMENT.autos();
        });
  }

  public Command update(LEDSegment... ledSegments) {
    return run(
        () -> {
          for (LEDSegment segment : ledSegments) {
            segment.update();
            for (int i = segment.startLED; i <= segment.endLED; i++) {
              buffer.setLED(i, segment.buffer.getLED(i - segment.startLED));
            }
          }
          led.setData(buffer);
        });
  }

  @Override
  public void close() throws Exception {
    led.close();
  }
}
