package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.GEARING;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_EXTENSION;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MIN_EXTENSION;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.SPROCKET_RADIUS;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.WEIGHT;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.sciborgs1155.robot.Constants;

public class SimElevator implements ElevatorIO {
  private final ElevatorSim elevator =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(
              DCMotor.getKrakenX60(2), WEIGHT.in(Kilograms), SPROCKET_RADIUS.in(Meters), GEARING),
          DCMotor.getKrakenX60(2),
          MIN_EXTENSION.in(Meters),
          MAX_EXTENSION.in(Meters),
          true,
          MIN_EXTENSION.in(Meters));

  public SimElevator() {
    elevator.update(0);
  }

  @Override
  /**
   * Sets the voltage for the elevator motor.
   *
   * @param voltage The voltage to set.
   */
  public void setVoltage(double voltage) {
    elevator.setInputVoltage(voltage);
    elevator.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  /**
   * Gets the current position of the elevator.
   *
   * @return The current position.
   */
  public double position() {
    return elevator.getPositionMeters();
  }

  /**
   * Gets the current velocity of the elevator.
   *
   * @return The current velocity.
   */
  @Override
  public double velocity() {
    return elevator.getVelocityMetersPerSecond();
  }

  @Override
  public void resetPosition() {
    elevator.setState(0, 0);
  }

  /**
   * Closes the elevator. This implementation does nothing.
   *
   * @throws Exception if an error occurs.
   */
  @Override
  public void close() throws Exception {}
}
