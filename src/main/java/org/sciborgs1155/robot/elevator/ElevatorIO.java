package org.sciborgs1155.robot.elevator;

public interface ElevatorIO extends AutoCloseable {
  public void setVoltage(double voltage);

  public double position();

  public double velocity();
}
