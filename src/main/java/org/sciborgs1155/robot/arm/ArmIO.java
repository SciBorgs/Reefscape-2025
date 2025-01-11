package org.sciborgs1155.robot.arm;

/** Hardware interface for {@link Arm} subsystem */
public interface ArmIO {

  /**
   * @return The position in radians.
   */
  public double position();

  /**
   * @return The position in radians/sec.
   */
  public double velocity();

  /** Sets the voltage of the arm motor. */
  public void setVoltage(double voltage);

  public void close();
}
