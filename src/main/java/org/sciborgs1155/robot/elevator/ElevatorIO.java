package org.sciborgs1155.robot.elevator;

public interface ElevatorIO extends AutoCloseable {
  /**
   * Sets elevator voltage.
   *
   * @param voltage Voltage inputted to gearbox.
   */
  public void setVoltage(double voltage);

  /**
   * Returns height of the elevator.
   *
   * @return The encoder value in meters.
   */
  public double position();

  /**
   * Returns the velocity of the elevator.
   *
   * @return The encoder value in meters per second.
   */
  public double velocity();

  /**
   * Resets the elevator encoder to a measurement of 0.
   */
  public void resetPosition();
}
