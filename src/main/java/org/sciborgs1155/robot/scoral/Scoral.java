package org.sciborgs1155.robot.scoral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Robot;

public class Scoral extends SubsystemBase {

  private ScoralIO scoral;

  public Scoral(ScoralIO scoral) {
    this.scoral = scoral;
    System.out.println("");
  }

  public static Scoral create() {
    return Robot.isReal() ? new Scoral(new RealScoral()) : new Scoral(new SimScoral());
  }

  public static Scoral none() {
    return new Scoral(new NoScoral());
  }

  public Command outtake() {
    return run(() -> scoral.outtake());
  }
}
