package org.sciborgs1155.robot.roller;

import com.ctre.phoenix6.hardware.TalonFX;

import static org.sciborgs1155.robot.Ports.GroundIntake.*;

public class RealRoller implements RollerIO {

      /** Controls intake power */
  private final TalonFX motor;
  //TODO this should not be a talonfx

    public RealRoller() {
        motor = new TalonFX(ARM_MOTOR);
    }

  @Override
  public void set(double power) {
  }

@Override
public void close() {
    motor.close();
}
}
