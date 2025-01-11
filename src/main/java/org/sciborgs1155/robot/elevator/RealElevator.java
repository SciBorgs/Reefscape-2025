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
  /**
   * Sets the voltage for the elevator motor.
   *
   * @param voltage The voltage to set.
   */
  public void setVoltage(double voltage) {
    lead.setVoltage(voltage);
  }

  @Override
  /**
   * Gets the current position of the elevator.
   *
   * @return The current position.
   */
  public double position() {
    return lead.getPosition().getValueAsDouble();
  }

  @Override
  /**
   * Gets the current velocity of the elevator.
   *
   * @return The current velocity.
   */
  public double velocity() {
    return lead.getVelocity().getValueAsDouble();
  }

  @Override
  /**
   * Closes the elevator.
   *
   * @throws Exception if an error occurs.
   */
  public void close() throws Exception {
    lead.close();
    follower.close();
  }
}
