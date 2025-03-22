package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.sciborgs1155.robot.Constants;

/** Simulated {@link ArmIO} hardware interface. */
public class SimArm implements ArmIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          GEARBOX,
          GEARING,
          MOI,
          ARM_LENGTH.in(Meters),
          MIN_ANGLE.in(Radians),
          MAX_ANGLE.in(Radians),
          true,
          DEFAULT_ANGLE.in(Radians));

  @Override
  public double position() {
    return sim.getAngleRads();
  }

  public SimArm() {
    sim.update(0);
  }

  @Override
  public double velocity() {
    return sim.getVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
    sim.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public void setCurrentLimit(Current limit) {}

  @Override
  public void close() {}
}
