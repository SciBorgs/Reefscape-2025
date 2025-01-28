package org.sciborgs1155.lib;

import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.BooleanSupplier;

/**
 * a beambreak wrapper that contains two main elements: 1. A BooleanSupplier detailing the
 * beambreak's state; true for unbroken, false for broken 2. A close runnable that will close the
 * resource if necessary
 */
public class Beambreak {
  private final BooleanSupplier beambreak;
  private final Runnable close;

  public Beambreak(BooleanSupplier beambreak, Runnable close) {
    this.beambreak = beambreak;
    this.close = close;
  }

  /**
   * generates a beambreak wrapper based off a channel
   *
   * @param channel the channel for the beambreak
   */
  public static Beambreak real(int channel) {
    DigitalInput beambreak = new DigitalInput(channel);
    return new Beambreak(() -> beambreak.get(), beambreak::close);
  }

  /** Generates a beambreak that does not have hardware This beambreak will always return true. */
  public static Beambreak none() {
    return new Beambreak(() -> true, () -> {});
  }

  /** the value of the beambreak; true for unbroken, false for broken */
  public boolean get() {
    return beambreak.getAsBoolean();
  }

  /** closes all resources if necessary */
  public void close() {
    close.run();
  }
}
