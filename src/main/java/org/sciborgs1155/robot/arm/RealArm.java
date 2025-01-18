package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Ports.GroundIntake.*;
import static org.sciborgs1155.robot.arm.ArmConstants.GEARING;
import static org.sciborgs1155.robot.arm.ArmConstants.STRATOR_LIMIT;
import static org.sciborgs1155.robot.arm.ArmConstants.SUPPLY_LIMIT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

/** {@link ArmIO} class with a {@link SparkMax} motor controller. */
public class RealArm implements ArmIO {
  /** Controls arm orientation. */
  private final TalonFX motor;

  private TalonFXConfiguration config;

  public RealArm() {
    motor = new TalonFX(ARM_MOTOR);

    // Resetting configuration
    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = STRATOR_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = GEARING;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.FeedbackRemoteSensorID = CANCODER;

    motor.getConfigurator().apply(config);

    FaultLogger.register(motor);
    TalonUtils.addMotor(motor);
  }

  @Override
  public double position() {
    // Converting 'rotations' to 'radians'
    return motor.getPosition().getValue().in(Radians);
  }

  @Override
  public double velocity() {
    // Converting 'rotations' to 'radians'
    return motor.getVelocity().getValue().in(RadiansPerSecond);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void currentLimit(double limit) {
    config.CurrentLimits.SupplyCurrentLimit = limit;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void close() {
    motor.close();
  }
}
