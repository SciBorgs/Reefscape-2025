package org.sciborgs1155.robot.scoral;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Scoral.*;
import static org.sciborgs1155.robot.scoral.ScoralConstants.CURRENT_LIMIT;
import static org.sciborgs1155.robot.scoral.ScoralConstants.STATOR_LIMIT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealScoral implements ScoralIO {
  TalonFX scoral = new TalonFX(SCORAL, "rio");
  TalonFX algae = new TalonFX(ALGAE, "rio");
  DigitalInput beambreak = new DigitalInput(BEAMBREAK);

  public RealScoral() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    FaultLogger.register(scoral);
    TalonUtils.addMotor(scoral);
    FaultLogger.register(algae);
    TalonUtils.addMotor(algae);
    scoral.getConfigurator().apply(config);
    algae.getConfigurator().apply(config);
  }

  @Override
  public boolean hasCoral() {
    return !beambreak.get();
  }

  @Override
  public void close() throws Exception {
    beambreak.close();
    scoral.close();
    algae.close();
  }

  @Override
  public void scoralPower(double power, boolean outtaking) {
    scoral.setVoltage(power);
  }

  @Override
  public void algaePower(double power, boolean outtaking) {
    algae.setVoltage(power);
  }
}
