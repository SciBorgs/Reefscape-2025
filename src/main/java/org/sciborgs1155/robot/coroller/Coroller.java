package org.sciborgs1155.robot.coroller;

import static org.sciborgs1155.robot.coroller.CorollerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

/** Simple roller subsystem used for intaking/outtaking coral. */
public class Coroller extends SubsystemBase implements Logged, AutoCloseable {
  /** Interface for interacting with the motor itself. */
  private final CorollerIO hardware;

  /**
   * Returns a new {@link Coroller} subsystem, which will have real hardware if the robot is real,
   * and no hardware connection if it isn't.
   */
  public static Coroller create() {
    return Robot.isReal() ? new Coroller(new RealCoroller()) : new Coroller(new NoCoroller());
  }

  /** Creates a new {@link Coroller} with no hardware interface(does nothing). */
  public static Coroller none() {
    return new Coroller(new NoCoroller());
  }

  private Coroller(CorollerIO hardware) {
    this.hardware = hardware;
  }

  /** Makes the roller spin inwards(towards robot). */
  public Command intake() {
    return run(() -> hardware.setPower(INTAKE_POWER)).withName("Intaking");
  }

  /** Makes the roller spin outwards(away from robot). */
  public Command outtake() {
    return run(() -> hardware.setPower(OUTTAKE_POWER)).withName("Outtaking");
  }

  /** Stops the roller motors. */
  public Command halt() {
    return run(() -> hardware.setPower(0));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
