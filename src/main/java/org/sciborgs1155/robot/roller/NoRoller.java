package org.sciborgs1155.robot.roller;

/** Disfunctional Placeholder for {@link RollerIO} (In cases where the Roller is unusable) */
public class NoRoller implements RollerIO {
  @Override
  public void setPower(double power) {}

  @Override
  public void close() {}
}
