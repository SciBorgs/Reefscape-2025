package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Ports.GroundIntake.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

/** {@link ArmIO} class with {@link TalonFX} motor controllers */
public class RealArm implements ArmIO {
  /** Controls arm orientation */
  private final TalonFX motor;

  public RealArm() {
    motor = new TalonFX(ARM_MOTOR);

    // Resetting configurations
    motor.getConfigurator().apply(new TalonFXConfiguration());
  }

  @Override
  public double position() {
    return motor.getPosition().getValue().in(Radians);
  }

  @Override
  public double velocity() {
    return motor.getVelocity().getValue().in(RadiansPerSecond);
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
