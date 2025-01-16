package org.sciborgs1155.robot.hopper;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Ports.Hopper;

public class RealHopper implements HopperIO {
  private final SimpleMotor motor;

  private final DigitalInput beambreak = new DigitalInput(Hopper.BEAMBREAK);

  public RealHopper() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = HopperConstants.CURRENT_LIMIT.in(Amps);

    motor = SimpleMotor.talon(new TalonFX(Hopper.MOTOR), config);
  }

  @Override
  public void set(double power) {
    motor.set(power);
  }

  @Override
  public boolean beambreak() {
    return beambreak.get();
  }

  @Override
  public void close() {
    motor.close();
    beambreak.close();
  }
}
