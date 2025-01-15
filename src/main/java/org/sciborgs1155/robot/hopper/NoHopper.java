package org.sciborgs1155.robot.hopper;

public class NoHopper implements HopperIO {
  @Override
  public void setVoltage(double voltage) {}

  @Override
  public boolean beambreak() {
    return false;
  }

  @Override
  public void close() {}
}
