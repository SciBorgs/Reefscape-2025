package org.sciborgs1155.robot.led;

import static org.sciborgs1155.robot.Ports.LEDs.*;
import static org.sciborgs1155.robot.led.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
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
