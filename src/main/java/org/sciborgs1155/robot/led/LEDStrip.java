package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Ports.LEDs.*;
import static org.sciborgs1155.robot.led.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public Command elevatorLED() {
    return runOnce(
            () -> {
              LEFT_LED_SEGMENT.music();
              RIGHT_LED_SEGMENT.rainbow();
            })
        .andThen(set(LED_SEGMENTS));
  }

  public Command test() {
    return runOnce(
            () -> {
              LEFT_LED_SEGMENT.music();
              RIGHT_LED_SEGMENT.rainbow();
            })
        .andThen(set(LED_SEGMENTS));
  }

  public Command set(LEDSegment... ledSegments) {
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
