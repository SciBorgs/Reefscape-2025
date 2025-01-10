package org.sciborgs1155.robot.scoral;

public class NoScoral implements ScoralIO {

  @Override
  public void setPower(double power) {}

  @Override
  public boolean beambreak() {
    return false;
  }

  @Override
  public void close() throws Exception {}
}
