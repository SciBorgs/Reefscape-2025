package org.sciborgs1155.robot.arm;

/** Hardware interface for {@link Arm} subsystem */
public interface ArmIO extends AutoCloseable {
  /**
   * @return The position in radians.
   */
  public double position();

  /**
   * @return The position in radians/sec.
   */
  public double velocity();

  /**
   * @return Returns the voltage of the arm motor.
   */
  public double voltage();

  /** Sets the voltage of the arm motor. */
  public void setVoltage(double voltage);
}
