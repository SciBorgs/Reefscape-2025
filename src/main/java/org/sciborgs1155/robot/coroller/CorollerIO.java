package org.sciborgs1155.robot.coroller;

public interface CorollerIO extends AutoCloseable {

  /**
   * Sets the roller to a certain power level.
   *
   * @param power The power level, from -1 to 1.
   */
  public void set(double power);
}
