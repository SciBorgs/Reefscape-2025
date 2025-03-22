package org.sciborgs1155.robot.elevator;

public interface ElevatorIO extends AutoCloseable {
  /**
   * Sets elevator voltage.
   *
   * @param voltage Voltage inputted to gearbox.
   */
  public void setVoltage(double voltage);

  /**
   * Sets elevator position, and moves to it.
   *
   * @param position Position to command the elevator to, in meters.
   */
  public void setGoal(double position);

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
}
