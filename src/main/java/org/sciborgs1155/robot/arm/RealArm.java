package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Ports.GroundIntake.*;
import static org.sciborgs1155.robot.arm.ArmConstants.GEARING;
import static org.sciborgs1155.robot.arm.ArmConstants.STRATOR_LIMIT;
import static org.sciborgs1155.robot.arm.ArmConstants.SUPPLY_LIMIT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Current;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

/** {@link ArmIO} class with a {@link SparkMax} motor controller. */
public class RealArm implements ArmIO {
  /** Controls arm orientation. */
  private final TalonFX leader;
  private final TalonFX follower;


  private TalonFXConfiguration leaderConfig;
  private TalonFXConfiguration followerConfig;

  public RealArm() {
    leader = new TalonFX(ARM_LEADER);
    follower = new TalonFX(ARM_FOLLOWER);

    // Resetting configuration
    leaderConfig = new TalonFXConfiguration();

    leaderConfig.CurrentLimits.StatorCurrentLimit = STRATOR_LIMIT.in(Amps);
    leaderConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leaderConfig.Feedback.SensorToMechanismRatio = GEARING;

    leaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    leaderConfig.Feedback.FeedbackRemoteSensorID = CANCODER;

    leader.getConfigurator().apply(leaderConfig);
    follower.setControl(new Follower(ARM_FOLLOWER, false));

    

    FaultLogger.register(leader);
    TalonUtils.addMotor(leader);

    FaultLogger.register(follower);
    TalonUtils.addMotor(follower);
  }

  @Override
  public double position() {
    // Converting 'rotations' to 'radians'
    return leader.getPosition().getValue().in(Radians);
  }

  @Override
  public double velocity() {
    // Converting 'rotations' to 'radians'
    return leader.getVelocity().getValue().in(RadiansPerSecond);
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  @Override
  public void setCurrentLimit(Current limit) {
    leaderConfig.CurrentLimits.SupplyCurrentLimit = limit.in(Amps);
    leader.getConfigurator().apply(leaderConfig);
  }

  @Override
  public void close() {
    leader.close();
  }
}
