package org.sciborgs1155.robot.hopper;

public class NoHopper implements HopperIO {
  @Override
  public void setPower(double power) {}

  @Override
  public boolean beambreak() {
    return false;
  }

  @Override
  public void close() {}
}
