package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Constants.CANIVORE_NAME;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.robot.Ports;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RealClimb implements ClimbIO {
  private final TalonFX talon;

  private TalonFXConfiguration config;

    public RealClimb() {
        talon = new TalonFX(Ports.Climb.CLIMB, CANIVORE_NAME);

        // Resetting configuration
        config = new TalonFXConfiguration();
    
        config.CurrentLimits.StatorCurrentLimit = 140;
        config.CurrentLimits.SupplyCurrentLimit = 140;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = ClimbConstants.GEARING;
    
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
        talon.getConfigurator().apply(config);
    
        FaultLogger.register(talon);
        TalonUtils.addMotor(talon);
    }

    @Override
    public void setVoltage(double voltage) {
        talon.setVoltage(voltage);
    }

    @Override
    public double position() {
        return talon.getPosition().getValue().in(Radians);
    }

    @Override
    public double velocity() {
        return talon.getVelocity().getValue().in(RadiansPerSecond);
    }

    @Override
    public void close() throws Exception {
        talon.close();
    }
}
