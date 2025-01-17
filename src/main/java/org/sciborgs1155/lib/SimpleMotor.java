package org.sciborgs1155.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import java.util.function.DoubleConsumer;

/**
 * Simple Motor controller that has a {@link #set} method and nothing else. Can work with {@link
 * TalonFX}'s and {@link SparkBase}'s.
 */
public class SimpleMotor {
  /** Interface for setting motor power. */
  private final DoubleConsumer set;

  /** Interface for closing the motor. */
  private final Runnable close;

  /**
   * Constructor.
   *
   * @param set : {@link DoubleConsumer} for setting motor power.
   * @param close : {@link Runnable} interface for the motor.
   */
  public SimpleMotor(DoubleConsumer set, Runnable close) {
    this.set = set;
    this.close = close;
  }

  /**
   * Returns a new {@link SimpleMotor} that controls a {@link TalonFX} motor. The motor is also
   * registered with {@link TalonUtils} and {@link FaultLogger}.
   */
  public static SimpleMotor talon(TalonFX motor, TalonFXConfiguration config) {
    FaultLogger.register(motor);
    TalonUtils.addMotor(motor);
    motor.getConfigurator().apply(config);
    return new SimpleMotor(motor::set, motor::close);
  }

  /**
   * Returns a new {@link SimpleMotor} that controls a {@link SparkBase} motor. The motor is also
   * registered with {@link FaultLogger}.
   */
  public static SimpleMotor spark(SparkBase motor, SparkBaseConfig config) {
    FaultLogger.register(motor);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    return new SimpleMotor(motor::set, motor::close);
  }

  /** Returns a new {@link SimpleMotor} that does absoluteley nothing. */
  public static SimpleMotor none() {
    return new SimpleMotor(v -> {}, () -> {});
  }

  /** Passes power into the {@link DoubleConsumer} specified in the constructor. */
  public void set(double power) {
    set.accept(power);
  }

  public void close() {
    close.run();
  }
}
