package org.sciborgs1155.robot.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Arm extends SubsystemBase implements Logged, AutoCloseable {
  private ArmIO hardware;

  public static Arm create() {
    return new NoArm();
  }

  public static Arm none() {
    return new NoArm();
  }

  /** Constructor */
  private Arm(ArmIO hardware) {
    this.hardware = hardware;
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
