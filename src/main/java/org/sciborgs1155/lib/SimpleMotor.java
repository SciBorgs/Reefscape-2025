package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Simple Motor controller that utilizes {@link DoubleSupplier}'s and {@link DoubleConsumer}'s. Can
 * work with {@link TalonFX}'s and {@link SparkBase}'s.
 */
public class SimpleMotor {
  /** Interface for setting motor power. */
  private final DoubleConsumer set;

  /** Interface for getting motor power. */
  private final DoubleSupplier get;

  /** Interface for closing the motor. */
  private final Runnable close;

  /**
   * Constructor.
   *
   * @param set : {@link DoubleConsumer} for setting motor power.
   * @param get : {@link DoubleSupplier} for getting the power of the motor.
   * @param close : {@link Runnable} interface for the motor.
   */
  public SimpleMotor(
      DoubleConsumer set,
      DoubleSupplier get,
      DoubleConsumer current,
      Runnable close) {
    this.set = set;
    this.get = get;
    this.close = close;
  }

  /**
   * Constructor.
   *
   * @param set : {@link DoubleConsumer} for setting motor power.
   * @param close : {@link Runnable} interface for the motor.
   */
  public SimpleMotor(DoubleConsumer set, Runnable close) {
    this.set = set;
    this.get = () -> 0;
    this.close = close;
  }

  /**
   * @return Returns a new {@link SimpleMotor} that controls a {@link TalonFX} motor. The motor is
   *     registered with {@link TalonUtils} and {@link FaultLogger}.
   * @param motor : {@link TalonFX} controller instance with device ID.
   * @param config : {@link TalonFXConfiguration} to apply to the motor.
   */
  public static SimpleMotor talon(TalonFX motor, TalonFXConfiguration config) {
    FaultLogger.register(motor);
    TalonUtils.addMotor(motor);
    motor.getConfigurator().apply(config);
    return new SimpleMotor(
        motor::set,
        motor::get,
        (current) ->
            motor
                .getConfigurator()
                .apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(current)),
        motor::close);
  }

  /**
   * @return Returns a new {@link SimpleMotor} that controls a {@link SparkBase} motor. The motor is
   *     registered with {@link FaultLogger}.
   * @param motor : {@link SparkBase} controller instance with device ID.
   * @param config : {@link SparkBaseConfig} to apply to the motor.
   */
  public static SimpleMotor spark(SparkBase motor, SparkBaseConfig config) {
    FaultLogger.register(motor);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    return new SimpleMotor(
        motor::set,
        motor::get,
        (current) -> motor.configureAsync(config.smartCurrentLimit((int) current), null, null),
        motor::close);
  }

  /** Returns a new {@link SimpleMotor} that does absoluteley nothing. */
  public static SimpleMotor none() {
    return new SimpleMotor(v -> {}, () -> {});
  }

  /** Passes power into the '{@link #set}' consumer specified in the constructor. */
  public void set(double power) {
    set.accept(power);
  }

  /**
   * @return The power from the '{@link #get}'' supplier specified in the constructor.
   */
  public double get() {
    return get.getAsDouble();
  }
  
  public void close() {
    close.run();
  }
}
