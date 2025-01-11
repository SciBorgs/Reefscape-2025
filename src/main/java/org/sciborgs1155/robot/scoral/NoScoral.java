package org.sciborgs1155.robot.scoral;

public class NoScoral implements ScoralIO {

  @Override
  public void setPower(double power) {}

  @Override
  public boolean beambreak() {
    return true;
  }

  @Override
  public void close() throws Exception {}
}
