package org.sciborgs1155.robot.roller;

import static org.sciborgs1155.robot.roller.RollerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

/** Simple roller subsystem used for intaking/outtaking coral. */
public class Roller extends SubsystemBase implements Logged, AutoCloseable {
  /** Interface for interacting with the motor itself. */
  private final RollerIO hardware;

  /**
   * Returns a new {@link Roller} subsystem, which will have real hardware if the robot is real, and
   * no hardware connection if it isn't.
   */
  public static Roller create() {
    return Robot.isReal() ? new Roller(new RealRoller()) : new Roller(new NoRoller());
  }

  /** Creates a new {@link Roller} with no hardware interface(does nothing). */
  public static Roller none() {
    return new Roller(new NoRoller());
  }

  private Roller(RollerIO hardware) {
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
