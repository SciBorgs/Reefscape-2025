package org.sciborgs1155.robot.elevator;

public class NoElevator implements ElevatorIO {

  @Override
  /**
   * Sets the voltage for the elevator motor. This implementation does nothing.
   *
   * @param voltage The voltage to set.
   */
  public void setVoltage(double voltage) {}

  @Override
  /**
   * Gets the current position of the elevator. This implementation always returns 0.
   *
   * @return The current position, always 0.
   */
  public double position() {
    return 0;
  }

  @Override
  /**
   * Gets the current velocity of the elevator. This implementation always returns 0.
   *
   * @return The current velocity, always 0.
   */
  public double velocity() {
    return 0;
  }

  @Override
  /**
   * Closes the elevator. This implementation does nothing.
   *
   * @throws Exception if an error occurs.
   */
  public void close() throws Exception {}

  @Override
  public void setGoal(double meters) {}
}
