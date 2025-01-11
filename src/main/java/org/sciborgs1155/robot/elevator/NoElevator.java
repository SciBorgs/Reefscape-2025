package org.sciborgs1155.robot.elevator;

public class NoElevator implements ElevatorIO {

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double position() {
    return 0;
  }

  @Override
  public double velocity() {
    return 0;
  }

  @Override
  public void close() throws Exception {}
}
