package org.sciborgs1155.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.led.LEDStrip;

/** Used to tell multiple LEDStrip to do things in a cleaner way. */
public class LEDing {
  private final LEDStrip leftStrip;
  private final LEDStrip middleStrip;
  private final LEDStrip rightStrip;

  public LEDing(LEDStrip leftStrip, LEDStrip middleStrip, LEDStrip rightStrip) {
    this.leftStrip = leftStrip;
    this.middleStrip = middleStrip;
    this.rightStrip = rightStrip;
  }

  /** Sets all LEDStrips to elevator progress gradient. */
  public Command progressGradient(DoubleSupplier percent, BooleanSupplier atGoal) {
    return leftStrip
        .progressGradient(percent, atGoal)
        .alongWith(rightStrip.progressGradient(percent, atGoal));
  }

  /** Blinks all LEDStrips with a given color. */
  public Command blink(Color color) {
    return leftStrip.blink(color).alongWith(middleStrip.blink(color), rightStrip.blink(color));
  }

  /** Sets all LEDStrips with a given color. */
  public Command solid(Color color) {
    return leftStrip.solid(color).alongWith(middleStrip.solid(color), rightStrip.solid(color));
  }

  /** Sets all the LEDs based on an error. */
  public Command error(DoubleSupplier error) {
    return leftStrip.error(error).alongWith(middleStrip.error(error), rightStrip.error(error));
  }
}
