package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.sciborgs1155.robot.Constants;

/** Simulated {@link ArmIO} hardware interface. */
public class SimArm implements ArmIO {
  private final SingleJointedArmSim simulation =
      new SingleJointedArmSim(
          GEARBOX,
          GEARING,
          MOI,
          ARM_LENGTH.in(Meters),
          MIN_ANGLE.in(Radians),
          MAX_ANGLE.in(Radians),
          true,
          STARTING_ANGLE.in(Radians));

  @Override
  public double position() {
    simulation.update(Constants.PERIOD.in(Seconds));
    return simulation.getAngleRads();
  }

  @Override
  public double velocity() {
    return simulation.getVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double voltage) {
    simulation.setInputVoltage(voltage);
    simulation.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public double voltage() {
    return simulation.getInput().elementSum();
  }

  @Override
  public void close() {}
}
