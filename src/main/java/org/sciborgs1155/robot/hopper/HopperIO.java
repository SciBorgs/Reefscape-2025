package org.sciborgs1155.robot.hopper;

public interface HopperIO extends AutoCloseable {
    void setPower(double power);
    boolean beambreak();
}