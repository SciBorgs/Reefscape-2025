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

  public void setS(double s);

  public void setV(double v);

  public void setA(double a);

  public void setG(double g);

  public void setP(double p);

  public void setI(double i);

  public void setD(double d);
}
