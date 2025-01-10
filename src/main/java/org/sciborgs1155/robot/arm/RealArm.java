package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Ports.GroundIntake.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

/** {@link ArmIO} class with {@link TalonFX} motor controllers */
public class RealArm implements ArmIO {
  /** Controls arm orientation */
  private final TalonFX pivotMotor;

  /** Controls intake power */
  private final TalonFX rollerMotor;

  @Override
  public double position() {
    return pivotMotor.getPosition().getValue().in(Radians);
  }

  @Override
  public double velocity() {
    return rollerMotor.getVelocity().getValue().in(RadiansPerSecond);
  }

  @Override
  public void setArmVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  public RealArm() {
    pivotMotor = new TalonFX(ARM_MOTOR);
    rollerMotor = new TalonFX(ROLLER_MOTOR);

    // Resetting configurations
    pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
    rollerMotor.getConfigurator().apply(new TalonFXConfiguration());
  }
}
