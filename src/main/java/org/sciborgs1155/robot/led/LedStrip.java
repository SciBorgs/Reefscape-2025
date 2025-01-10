package org.sciborgs1155.robot.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

import static org.sciborgs1155.robot.Ports.LEDs.*;

public class LedStrip extends SubsystemBase implements Logged, AutoCloseable{

    private final AddressableLED led = new AddressableLED(LED_PORT);

    @Override
    public void close() throws Exception {
        led.close();
    }
    
}
