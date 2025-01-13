package org.sciborgs1155.robot.arm;

/** Disfunctional Placeholder for {@link ArmIO} (In cases where the Arm is unusable) */
public class NoArm implements ArmIO {
  @Override
  public double position() {
    return 0;
  }

  @Override
  public double velocity() {
    return 0;
  }

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void currentLimit(double limit) {}

  @Override
  public void close() {}
}
