package org.sciborgs1155.robot.arm;

/** Disfunctional Placeholder {@link ArmIO} */
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
  public void setVoltage(double voltage) {
    throw new UnsupportedOperationException("Unimplemented method 'setArmVoltage'");
  }

  @Override
  public void close() {}
}
