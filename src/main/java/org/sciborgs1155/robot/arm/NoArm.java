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
  public void setArmVoltage(double voltage) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setArmVoltage'");
  }
}
