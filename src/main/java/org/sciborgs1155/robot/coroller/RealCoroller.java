package org.sciborgs1155.robot.coroller;

import static org.sciborgs1155.robot.Ports.GroundIntake.*;

import com.ctre.phoenix6.hardware.TalonFX;

/** {@link CorollerIO} class with a {@link SparkMax} motor controller. */
public class RealCoroller implements CorollerIO {
  /** Controls roller speed */
  private final TalonFX motor;

  public RealCoroller() {
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
