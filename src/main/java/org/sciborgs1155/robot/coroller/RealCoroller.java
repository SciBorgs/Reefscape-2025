package org.sciborgs1155.robot.coroller;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.GroundIntake.*;
import static org.sciborgs1155.robot.coroller.CorollerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

/** {@link CorollerIO} class with a {@link SparkMax} motor controller. */
public class RealCoroller implements CorollerIO {
  /** Controls roller speed */
  private final TalonFX motor;

  private final TalonFXConfiguration config;

  public RealCoroller() {
    motor = new TalonFX(ROLLER_MOTOR);

    config = new TalonFXConfiguration();

    // TODO change one of these
    config.CurrentLimits.StatorCurrentLimit = STRATOR_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
    motor.getConfigurator().apply(config);

    FaultLogger.register(motor);
    TalonUtils.addMotor(motor);
  }

  @Override
  public void setPower(double power) {
    motor.set(power);
  }

  @Override
  public void close() {
    motor.close();
  }
}
