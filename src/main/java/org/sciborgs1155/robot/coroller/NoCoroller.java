package org.sciborgs1155.robot.coroller;

/** Disfunctional Placeholder for {@link CorollerIO} (In cases where the Roller is unusable) */
public class NoCoroller implements CorollerIO {
  @Override
  public void setPower(double power) {}

  @Override
  public void close() {}
}
