package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.lib.FaultLogger.register;
import static org.sciborgs1155.robot.Ports.Elevator.*;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.sciborgs1155.lib.TalonUtils;

public class RealElevator implements ElevatorIO {
  // idk how many there are
  private final TalonFX leader = new TalonFX(LEADER);
  private final TalonFX follower = new TalonFX(FOLLOWER);

  public RealElevator() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    follower.setControl(new Follower(LEADER, true));

    talonConfig.Feedback.FeedbackRemoteSensorID = CANCODER;
    talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.Feedback.SensorToMechanismRatio = CONVERSION_FACTOR;
    talonConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);

    leader.getConfigurator().apply(talonConfig);
    follower.getConfigurator().apply(talonConfig);

    TalonUtils.addMotor(leader);
    TalonUtils.addMotor(follower);
    register(leader);
    register(follower);
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  @Override
  public double position() {
    return leader.getPosition().getValueAsDouble();
  }

  @Override
  public double velocity() {
    return leader.getVelocity().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    leader.close();
    follower.close();
  }
}
