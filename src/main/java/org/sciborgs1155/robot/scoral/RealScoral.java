package org.sciborgs1155.robot.scoral;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Ports.Scoral.*;
import static org.sciborgs1155.robot.scoral.ScoralConstants.CURRENT_LIMIT;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealScoral implements ScoralIO {

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final DigitalInput beambreak;

  public RealScoral() {
    topMotor = new TalonFX(TOP_ROLLER);
    bottomMotor = new TalonFX(BOTTOM_ROLLER);

    beambreak = new DigitalInput(BEAMBREAK);

    TalonFXConfigurator topConfig = topMotor.getConfigurator();
    TalonFXConfigurator bottomConfig = topMotor.getConfigurator();

    CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
    limits.StatorCurrentLimit = CURRENT_LIMIT.in(Amps);
    limits.StatorCurrentLimitEnable = true;

    topConfig.apply(limits);
    bottomConfig.apply(limits);

    bottomMotor.setControl(new Follower(TOP_ROLLER, true));

    FaultLogger.register(topMotor);
    FaultLogger.register(bottomMotor);

    TalonUtils.addMotor(topMotor);
    TalonUtils.addMotor(bottomMotor);
  }

  @Override
  public void setPower(double power) {
    topMotor.set(power);
  }

  @Override
  public boolean beambreak() {
    return beambreak.get();
  }

  @Override
  public void close() throws Exception {
    topMotor.close();
    bottomMotor.close();
  }

  @Override
  public double getAngularVelocity() {
    return topMotor.getVelocity().getValue().in(RadiansPerSecond);
  }
}
