package org.sciborgs1155.robot.elevator;

import static org.sciborgs1155.robot.Ports.Elevator.*;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class RealElevator implements ElevatorIO {
  // idk how many there are
  private final TalonFX lead, follower;

  public RealElevator() {
    lead = new TalonFX(LEADER);
    follower = new TalonFX(FOLLOWER);

    follower.setControl(new Follower(LEADER, false));
  }

  @Override
  public void setVoltage(double voltage) {
    lead.setVoltage(voltage);
  }

  @Override
  public double position() {
    return lead.getPosition().getValueAsDouble();
  }

  @Override
  public double velocity() {
    return lead.getVelocity().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    lead.close();
    follower.close();
  }
}
