package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.sciborgs1155.robot.Constants;

public class SimElevator implements ElevatorIO {
  private final ElevatorSim elevator;

  public SimElevator() {
    elevator =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60(2), WEIGHT.in(Kilograms), DRUM_RADIUS.in(Meters), GEARING),
            DCMotor.getKrakenX60(2),
            MIN_HEIGHT.in(Meters),
            MAX_HEIGHT.in(Meters),
            true,
            MIN_HEIGHT.in(Meters));
  }

  @Override
  public void setVoltage(double voltage) {
    elevator.setInputVoltage(voltage);
    elevator.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public double position() {
    return elevator.getPositionMeters();
  }

  @Override
  public double velocity() {
    return elevator.getVelocityMetersPerSecond();
  }

  @Override
  public void close() throws Exception {}
}
