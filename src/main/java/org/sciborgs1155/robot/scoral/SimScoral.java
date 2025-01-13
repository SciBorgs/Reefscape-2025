package org.sciborgs1155.robot.scoral;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.scoral.ScoralConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimScoral implements ScoralIO {

  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(kV, kA), DCMotor.getKrakenX60(1));

  @Override
  public void setPower(double power) {
    sim.setInputVoltage(power);
    sim.update(PERIOD.in(Seconds));
  }

  @Override
  public boolean beambreak() {
    return true;
  }

  @Override
  public void close() throws Exception {}

  @Override
  public double getAngularVelocity() {
    return sim.getAngularVelocityRadPerSec();
  }
}
