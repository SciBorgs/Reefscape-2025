package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Constants.CANIVORE_NAME;
import static org.sciborgs1155.robot.Ports.GroundIntake.*;
import static org.sciborgs1155.robot.arm.ArmConstants.CANCODER_GEARING;
import static org.sciborgs1155.robot.arm.ArmConstants.STATOR_LIMIT;
import static org.sciborgs1155.robot.arm.ArmConstants.SUPPLY_LIMIT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

/** {@link ArmIO} class with a {@link SparkMax} motor controller. */
public class RealArm implements ArmIO {
  /** Controls arm orientation. */
  private final TalonFX leader;

  private TalonFXConfiguration config;

  public RealArm() {
    leader = new TalonFX(ARM_PIVOT, CANIVORE_NAME);

    // Resetting configuration
    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = CANCODER_GEARING;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.FeedbackRemoteSensorID = CANCODER;

    leader.getConfigurator().apply(config);

    FaultLogger.register(leader);
    TalonUtils.addMotor(leader);
  }

  @Override
  /** arm angle in radians */
  public double position() {
    // Converting 'rotations' to 'radians'
    return leader.getPosition().getValue().in(Radians);
  }

  @Override
  /** rotational velocity of arm in radians per second */
  public double velocity() {
    // Converting 'rotations' to 'radians'
    return leader.getVelocity().getValue().in(RadiansPerSecond);
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  @Override
  public void close() {
    leader.close();
  }
}
