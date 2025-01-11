package org.sciborgs1155.robot.roller;

/** Hardware interface for {@link Roller} subsystem */
public interface RollerIO extends AutoCloseable {
  /** Power is from -1 to 1(negative values reverse direction) */
  public void setPower(double power);
}
