package org.sciborgs1155.robot.roller;

import static org.sciborgs1155.robot.Ports.GroundIntake.*;

import com.ctre.phoenix6.hardware.TalonFX;

/** {@link RollerIO} class with a {@link SparkMax} motor controller. */
public class RealRoller implements RollerIO {
  /** Controls roller speed */
  private final TalonFX motor;

  public RealRoller() {
    motor = new TalonFX(ROLLER_MOTOR);
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
