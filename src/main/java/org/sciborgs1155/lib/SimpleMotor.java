package org.sciborgs1155.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.DoubleConsumer;

public class SimpleMotor {
  private final DoubleConsumer set;
  private final Runnable close;

  public SimpleMotor(DoubleConsumer set, Runnable close) {
    this.set = set;
    this.close = close;
  }

  public static SimpleMotor talon(TalonFX talon, TalonFXConfiguration config) {
    FaultLogger.register(talon);
    TalonUtils.addMotor(talon);
    talon.getConfigurator().apply(config);
    return new SimpleMotor(talon::set, talon::close);
  }

  public static SimpleMotor none() {
    return new SimpleMotor(v -> {}, () -> {});
  }

  public void set(double power) {
    set.accept(power);
  }

  public void close() {
    close.run();
  }
}
