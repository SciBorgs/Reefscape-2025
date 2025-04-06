package org.sciborgs1155.robot.climb;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Robot;

@Logged
public class Climb extends SubsystemBase implements AutoCloseable {
  private final ClimbIO hardware;

  public static Climb create() {
    return new Climb(Robot.isReal() ? new RealClimb() : new NoClimb());
  }

  public static Climb none() {
    return new Climb(new NoClimb());
  }

  public Climb(ClimbIO hardware) {
    this.hardware = hardware;

    setDefaultCommand(run(() -> hardware.setVoltage(0)).withName("stoooop"));
  }

  public Command climb() {
    return run(() -> hardware.setVoltage(ClimbConstants.CLIMB_VOLTAGE)).withName("climb");
  }

  public Command back() {
    return run(() -> hardware.setVoltage(ClimbConstants.BACK_VOLTAGE)).withName("back");
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
