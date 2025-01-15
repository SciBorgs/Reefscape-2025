package org.sciborgs1155.robot.elevator;

public interface ElevatorIO extends AutoCloseable {
  /**
   * sets elevator voltage
   *
   * @param voltage voltage inputed to gearbox
   */
  public void setVoltage(double voltage);

  /**
   * returns height of the elevator
   *
   * @return the encoder value in meters
   */
  public double position();

  /**
   * returns the velocity of the elevator
   *
   * @return the encoder value in meters per second
   */
  public double velocity();
}
