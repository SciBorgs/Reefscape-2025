package org.sciborgs1155.robot.hopper;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.robot.Ports.Hopper;

public class RealHopper implements HopperIO {
  private final TalonFX leftMotor = new TalonFX(Hopper.LEFT_MOTOR);
  private final TalonFX rightMotor = new TalonFX(Hopper.RIGHT_MOTOR);
  private final DigitalInput beambreak = new DigitalInput(Hopper.BEAMBREAK);

  public RealHopper() {
    TalonFXConfigurator configurator = rightMotor.getConfigurator();
    MotorOutputConfigs config = new MotorOutputConfigs();
    config.Inverted =
        InvertedValue
            .CounterClockwise_Positive; // DETERMINE INVERSION FROM CAD (WHICH IS NOT DONE YET)
    configurator.apply(config);

    FaultLogger.register(leftMotor);
    FaultLogger.register(rightMotor);

    TalonUtils.addMotor(leftMotor);
    TalonUtils.addMotor(rightMotor);
  }

  @Override
  public void setVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  @Override
  public boolean beambreak() {
    return beambreak.get();
  }

  @Override
  public void close() {
    leftMotor.close();
    rightMotor.close();
    beambreak.close();
  }
}
