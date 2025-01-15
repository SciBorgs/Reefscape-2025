package org.sciborgs1155.robot.hopper;

public interface HopperIO extends AutoCloseable {
  void setVoltage(double voltage);

  boolean beambreak();
}
