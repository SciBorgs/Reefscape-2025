package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.lib.FaultLogger.register;
import static org.sciborgs1155.robot.Constants.CANIVORE_NAME;
import static org.sciborgs1155.robot.Ports.Elevator.*;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.TalonUtils;

public class RealElevator implements ElevatorIO {
  private final TalonFX leader = new TalonFX(FRONT_LEADER, CANIVORE_NAME);
  private final TalonFX follower = new TalonFX(BACK_FOLLOWER, CANIVORE_NAME);

  private final MotionMagicVoltage profile = new MotionMagicVoltage(MIN_EXTENSION.in(Meters));

  public RealElevator() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    follower.setControl(new Follower(FRONT_LEADER, true));

    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.Feedback.SensorToMechanismRatio = CONVERSION_FACTOR;
    talonConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);

    talonConfig.MotionMagic.MotionMagicAcceleration =
        MAX_ACCEL.baseUnitMagnitude() / CONVERSION_FACTOR; // must be in rot/s^2
    talonConfig.MotionMagic.MotionMagicExpo_kA = 3;
    talonConfig.MotionMagic.MotionMagicJerk = 0 / CONVERSION_FACTOR;

    talonConfig.Slot0.kS = kS;
    talonConfig.Slot0.kG = kG;
    talonConfig.Slot0.kV = kV;
    talonConfig.Slot0.kA = kA;
    talonConfig.Slot0.kP = kP;
    talonConfig.Slot0.kI = kI;
    talonConfig.Slot0.kD = kD;
    talonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    leader.getConfigurator().apply(talonConfig);
    follower.getConfigurator().apply(talonConfig);

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

  public void setGoal(double position) {
    leader.setControl(profile.withPosition(position / CONVERSION_FACTOR).withSlot(0));
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
