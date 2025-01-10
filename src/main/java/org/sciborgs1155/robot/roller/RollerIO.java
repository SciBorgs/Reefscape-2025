package org.sciborgs1155.robot.roller;

public interface RollerIO {

  /**
   * Sets the power of the roller motor.
   *
   * @param power A number from -1 to 1 that represents the amount of power going into the motor
   */
  public void set(double power);

  public void close();
}
