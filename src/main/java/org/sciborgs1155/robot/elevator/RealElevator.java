package org.sciborgs1155.robot.elevator;

import static org.sciborgs1155.robot.Ports.Elevator.*;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import org.sciborgs1155.lib.TalonUtils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RealElevator implements ElevatorIO {
  // idk how many there are
  private final TalonFX leader = new TalonFX(LEADER);
  private final TalonFX follower = new TalonFX(FOLLOWER);
  

  public RealElevator() {
    TalonFXConfiguration drumConfig = new TalonFXConfiguration();
    follower.setControl(new Follower(LEADER, false));

    drumConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    drumConfig.Feedback.SensorToMechanismRatio = CONVERSION_FACTOR;
  
    leader.getConfigurator().apply(drumConfig);
    TalonUtils.addMotor(leader);
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  @Override
  public double position() {
    return leader.getPosition().getValueAsDouble() * CONVERSION_FACTOR;
  }

  @Override
  public double velocity() {
    return leader.getVelocity().getValueAsDouble() * CONVERSION_FACTOR;
  }

  @Override
  public void close() throws Exception {
    leader.close();
    follower.close();
  }
}
