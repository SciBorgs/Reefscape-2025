package org.sciborgs1155.robot.coroller;

/** Hardware interface for {@link Coroller} subsystem */
public interface CorollerIO extends AutoCloseable {
  /** Power is from -1 to 1(negative values reverse direction) */
  public void setPower(double power);
}
