package org.sciborgs1155.robot.arm;

import static org.sciborgs1155.robot.Ports.GroundIntake.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** {@link ArmIO} class with a {@link SparkMax} motor controller. */
public class RealArm implements ArmIO {
  /** Controls arm orientation. */
  private final SparkMax motor;

  /** Reads and stores position/velocity of the motor. */
  private final AbsoluteEncoder encoder;

  public RealArm() {
    motor = new SparkMax(ARM_MOTOR, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();

    // Resetting configuration
    motor.configure(
        new SparkMaxConfig().idleMode(IdleMode.kCoast),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public double position() {
    // Converting 'rotations' to 'radians'
    return encoder.getPosition() * Math.PI * 2;
  }

  @Override
  public double velocity() {
    // Converting 'rotations' to 'radians'
    return encoder.getVelocity() * Math.PI * 2;
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
