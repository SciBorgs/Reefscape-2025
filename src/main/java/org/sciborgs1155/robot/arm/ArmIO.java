package org.sciborgs1155.robot.arm;

import edu.wpi.first.units.measure.Current;
import monologue.Logged;

/** Hardware interface for {@link Arm} subsystem */
public interface ArmIO extends AutoCloseable, Logged {
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

  /**
   * Changes the current limit of the arm motor.
   *
   * @param limit The limit, in amps.
   */
  public void setCurrentLimit(Current limit);
}
