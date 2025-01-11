package org.sciborgs1155.robot.roller;

import static org.sciborgs1155.robot.Ports.GroundIntake.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/** {@link RollerIO} class with a {@link SparkMax} motor controller. */
public class RealRoller implements RollerIO {
  /** Controls roller speed */
  private final SparkMax motor;

  public RealRoller() {
    motor = new SparkMax(ROLLER_MOTOR, MotorType.kBrushless);
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
