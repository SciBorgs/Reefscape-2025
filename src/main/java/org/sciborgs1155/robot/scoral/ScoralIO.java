package org.sciborgs1155.robot.scoral;

public interface ScoralIO extends AutoCloseable {
  /**
   * Sets the motor power to a given percentage
   *
   * @param power the percent power from -1 to 1
   * @param outtaking indicates whether the current action taken by the scoral is outtaking or not
   */
  void set(double power, boolean outtaking);

  /**
   * @return a boolean indicating whether there is a coral in the scoral mechanism
   */
  boolean hasCoral();
}
