package org.sciborgs1155.robot.elevator;

public interface ElevatorIO extends AutoCloseable {
  public void setVoltage(double voltage);

  public double getPosition();

  public double getVelocity();
}
