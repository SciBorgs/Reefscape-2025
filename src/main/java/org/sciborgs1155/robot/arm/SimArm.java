package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Simulated {@link ArmIO} hardware interface. */
public class SimArm implements ArmIO {
  private final SingleJointedArmSim simulation = new SingleJointedArmSim(
      GEARBOX,
      GEARING,
      MOI,
      ARM_LENGTH.in(Meters),
      MIN_ANGLE.in(Radians),
      MAX_ANGLE.in(Radians),
      true,
      STARTING_ANGLE.getRadians());

  private Time previousUpdateTime = Seconds.of(0);

  @Override
  public double position() {
    simulation.update(Timer.getFPGATimestamp() - previousUpdateTime.in(Seconds));
    previousUpdateTime = Seconds.of(Timer.getFPGATimestamp());
    return simulation.getAngleRads();
  }

  @Override
  public double velocity() {
    return simulation.getVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double voltage) {
    simulation.setInputVoltage(voltage);
  }

  @Override
  public double voltage() {
    return simulation.getInput().elementSum();
  }

  @Override
  public void close() {
  }
}
