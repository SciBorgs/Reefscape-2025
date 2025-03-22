package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import monologue.Annotations.Log;
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

  @Log.NT
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(MetersPerSecond), MAX_ACCEL.in(MetersPerSecondPerSecond)));

  @Log.NT private final ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

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

  /**
   * Closes the elevator. This implementation does nothing.
   *
   * @throws Exception if an error occurs.
   */
  @Override
  public void close() throws Exception {}

  @Override
  public void setGoal(double position) {
    double goal =
        Double.isNaN(position)
            ? MIN_EXTENSION.in(Meters)
            : MathUtil.clamp(position, MIN_EXTENSION.in(Meters), MAX_EXTENSION.in(Meters));
    double lastVelocity = pid.getSetpoint().velocity;
    double feedback = pid.calculate(position(), goal);
    double feedforward = ff.calculateWithVelocities(lastVelocity, pid.getSetpoint().velocity);

    setVoltage(feedforward + feedback);
  }
}
