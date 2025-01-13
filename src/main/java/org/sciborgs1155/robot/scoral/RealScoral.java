package org.sciborgs1155.robot.scoral;

import static org.sciborgs1155.robot.Ports.Scoral.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.sciborgs1155.lib.FaultLogger;

public class RealScoral implements ScoralIO {

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final DigitalInput beambreak;

  public RealScoral() {
    topMotor = new TalonFX(TOP_ROLLER);
    bottomMotor = new TalonFX(BOTTOM_ROLLER);

    beambreak = new DigitalInput(BEAMBREAK);

    bottomMotor.setControl(new Follower(TOP_ROLLER, true));

    FaultLogger.register(topMotor);
    FaultLogger.register(bottomMotor);
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
}
