package org.sciborgs1155.lib;

import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.BooleanSupplier;

public class Beambreak {
  private final BooleanSupplier beambreak;
  private final Runnable close;

  public Beambreak(BooleanSupplier beambreak, Runnable close) {
    this.beambreak = beambreak;
    this.close = close;
  }

  public static Beambreak real(int channel) {
    DigitalInput beambreak = new DigitalInput(channel);
    return new Beambreak(() -> beambreak.get(), beambreak::close);
  }

  public static Beambreak none() {
    return new Beambreak(() -> true, () -> {});
  }

  public boolean get() {
    return beambreak.getAsBoolean();
  }

  public void close() {
    close.run();
  }
}
