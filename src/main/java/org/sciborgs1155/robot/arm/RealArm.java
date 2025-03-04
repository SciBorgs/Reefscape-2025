package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Constants.CANIVORE_NAME;
import static org.sciborgs1155.robot.Ports.GroundIntake.*;
import static org.sciborgs1155.robot.arm.ArmConstants.GEARING;
import static org.sciborgs1155.robot.arm.ArmConstants.STATOR_LIMIT;
import static org.sciborgs1155.robot.arm.ArmConstants.SUPPLY_LIMIT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Current;
import monologue.Annotations.Log;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

/** {@link ArmIO} class with a {@link SparkMax} motor controller. */
public class RealArm implements ArmIO {
  /** Controls arm orientation. */
  private final TalonFX leader;

  // private final TalonFX follower;

  private TalonFXConfiguration config;

  public RealArm() {
    leader = new TalonFX(ARM_LEADER, CANIVORE_NAME);
    // follower = new TalonFX(ARM_FOLLOWER, CANIVORE_NAME);

    // follower.setControl(new Follower(ARM_LEADER, false));

    // Resetting configuration
    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = GEARING;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // config.Feedback.FeedbackRemoteSensorID = CANCODER;

    leader.getConfigurator().apply(config);
    // follower.getConfigurator().apply(config);

    FaultLogger.register(leader);
    TalonUtils.addMotor(leader);

    // FaultLogger.register(follower);
    // TalonUtils.addMotor(follower);
  }

  @Override
  @Log.NT
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
    config.CurrentLimits.SupplyCurrentLimit = limit.in(Amps);
    leader.getConfigurator().apply(config);
  }

  @Override
  public void close() {
    leader.close();
  }
}
