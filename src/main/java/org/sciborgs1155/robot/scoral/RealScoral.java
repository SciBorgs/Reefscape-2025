package org.sciborgs1155.robot.scoral;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Scoral.BEAMBREAK;
import static org.sciborgs1155.robot.Ports.Scoral.ROLLER;
import static org.sciborgs1155.robot.scoral.ScoralConstants.CURRENT_LIMIT;
import static org.sciborgs1155.robot.scoral.ScoralConstants.STATOR_LIMIT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealScoral implements ScoralIO {
  TalonFX motor = new TalonFX(ROLLER, "rio");
  DigitalInput beambreak = new DigitalInput(BEAMBREAK);

  public RealScoral() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    FaultLogger.register(motor);
    TalonUtils.addMotor(motor);
    motor.getConfigurator().apply(config);
  }

  @Override
  public void set(double power, boolean outtaking) {
    motor.set(power);
  }

  @Override
  public boolean hasCoral() {
    return !beambreak.get();
  }

  @Override
  public void close() throws Exception {
    beambreak.close();
    motor.close();
  }
}
