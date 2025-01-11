package org.sciborgs1155.robot.elevator;

public interface ElevatorIO extends AutoCloseable {
  /**
   * Sets the voltage for the elevator motor.
   *
   * @param voltage The voltage to set.
   */
  public void setVoltage(double voltage);

  /**
   * Gets the current position of the elevator.
   *
   * @return The current position.
   */
  public double position();

  /**
   * Gets the current velocity of the elevator.
   *
   * @return The current velocity.
   */
  public double velocity();
}
