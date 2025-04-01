package org.sciborgs1155.robot.scoral;

public class NoScoral implements ScoralIO {
  @Override
  public boolean hasCoral() {
    return false;
  }

  @Override
  public void close() throws Exception {}

  @Override
  public void scoralPower(double power, boolean outtaking) {}

  @Override
  public void algaePower(double power, boolean outtaking) {}
}
