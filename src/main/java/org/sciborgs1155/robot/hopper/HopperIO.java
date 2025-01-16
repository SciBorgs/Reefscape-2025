package org.sciborgs1155.robot.hopper;

public interface HopperIO extends AutoCloseable {
  void set(double power);

  boolean beambreak();
}
