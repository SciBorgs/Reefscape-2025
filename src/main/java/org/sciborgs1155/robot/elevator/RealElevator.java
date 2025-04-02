package org.sciborgs1155.robot.elevator;

import static org.sciborgs1155.lib.FaultLogger.register;
import org.sciborgs1155.lib.TalonUtils;
import static org.sciborgs1155.robot.Constants.CANIVORE_NAME;
import static org.sciborgs1155.robot.Ports.Elevator.BACK_FOLLOWER;
import static org.sciborgs1155.robot.Ports.Elevator.FRONT_LEADER;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.CONVERSION_FACTOR;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.CURRENT_LIMIT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;

public class RealElevator implements ElevatorIO {
  private final TalonFX leader = new TalonFX(FRONT_LEADER, CANIVORE_NAME);
  private final TalonFX follower = new TalonFX(BACK_FOLLOWER, CANIVORE_NAME);

  public RealElevator() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    follower.setControl(new Follower(FRONT_LEADER, true));

    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.Feedback.SensorToMechanismRatio = CONVERSION_FACTOR;
    talonConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);

    leader.getConfigurator().apply(talonConfig);
    follower.getConfigurator().apply(talonConfig);

    leader.setPosition(0);
    follower.setPosition(0);

    TalonUtils.addMotor(leader);
    TalonUtils.addMotor(follower);
    register(leader);
    register(follower);
  }

  @Override
  /**
   * Sets the voltage for the elevator motor.
   *
   * @param voltage The voltage to set.
   */
  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  @Override
  /**
   * Gets the current position of the elevator.
   *
   * @return The current position.
   */
  public double position() {
    return leader.getPosition().getValueAsDouble();
  }

  @Override
  /**
   * Gets the current velocity of the elevator.
   *
   * @return The current velocity.
   */
  public double velocity() {
    return leader.getVelocity().getValueAsDouble();
  }

  @Override
  public void resetPosition() {
    leader.setPosition(0);
  }

  @Override
  /**
   * Closes the elevator.
   *
   * @throws Exception if an error occurs.
   */
  public void close() throws Exception {
    leader.close();
    follower.close();
  }
}
