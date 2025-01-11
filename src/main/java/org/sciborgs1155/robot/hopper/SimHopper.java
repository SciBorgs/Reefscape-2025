package org.sciborgs1155.robot.hopper;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.sciborgs1155.robot.Constants;

public class SimHopper implements HopperIO {
  private final DCMotorSim leftSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(HopperConstants.kV, HopperConstants.kA),
          DCMotor.getNeoVortex(1),
          HopperConstants.GEARING);
  private final DCMotorSim rightSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(HopperConstants.kV, HopperConstants.kA),
          DCMotor.getNeoVortex(1),
          HopperConstants.GEARING);

  @Override
  public void setPower(double power) {
    leftSim.setInputVoltage(power);
    leftSim.update(Constants.PERIOD.in(Seconds));

    rightSim.setInputVoltage(-power); // one of these motors are supposed to be inverted.
    leftSim.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public boolean beambreak() {
    return true;
  }

  @Override
  public void close() throws Exception {}
}
