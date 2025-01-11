package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Ports.GroundIntake.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

/** {@link ArmIO} class with a {@link SparkMax} motor controller. */
public class RealArm implements ArmIO {
  /** Controls arm orientation. */
  private final TalonFX motor;

  public RealArm() {
    motor = new TalonFX(ARM_MOTOR);

    // Resetting configuration
    motor.getConfigurator().apply(new TalonFXConfiguration());
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
  public double voltage() {
    return motor.getMotorVoltage().getValue().in(Volts);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void close() {
    motor.close();
  }
}
