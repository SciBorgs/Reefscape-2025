package org.sciborgs1155.robot.scoral;

public class NoScoral implements ScoralIO {
  @Override
  public void set(double power, boolean outtaking) {}

  @Override
  public boolean hasCoral() {
    return false;
  }

  @Override
  public void close() throws Exception {}
}
