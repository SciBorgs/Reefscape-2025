package org.sciborgs1155.robot.coroller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coroller extends SubsystemBase {
  public static Coroller create() {
    return new Coroller();
  }

  /** Runs the coroller inward at a preset power. */
  public Command intake() {
    return run(() -> System.out.println("Hi my name is Ms Coroller and im intaking rn"));
  }

  /** Runs the coroller outward at a preset power. */
  public Command outtake() {
    return run(() -> System.out.println("Hi my name is Ms Coroller and im outtaking rn"));
  }
}
