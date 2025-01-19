package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

/** Simple Arm subsystem used for climbing and intaking coral from the ground. */
public class Arm extends SubsystemBase implements Logged, AutoCloseable {
  /** Interface for interacting with the motor itself. */
  @Log.NT private final ArmIO hardware;

  /**
   * Returns a new {@link Arm} subsystem, which will have real hardware if the robot is real, and
   * simulated if it isn't.
   */
  public static Arm create() {
    return new Arm(Robot.isReal() ? new RealArm() : new SimArm());
  }

  /** Creates a new {@link Arm} with no hardware interface(does nothing). */
  public static Arm none() {
    return new Arm(new NoArm());
  }

  /**
   * Constructor.
   *
   * @param hardware : The ArmIO object (real/simulated/nonexistent) that will be operated on.
   */
  private Arm(ArmIO hardware) {
    this.hardware = hardware;
  }

  /**
   * @return The position in radians.
   */
  @Log.NT
  public double position() {
    return hardware.position();
  }

  /**
   * @return True if the arm's position is close enough to its goal, False if it isn't.
   */
  public boolean atGoal() {
    return true;
  }

  /**
   * @return The position in radians/sec.
   */
  @Log.NT
  public double velocity() {
    return hardware.velocity();
  }

  @Log.NT
  /** Moves the arm towards a specified goal angle. */
  public Command goTo(Angle goal) {
    return run(() -> hardware.setVoltage(0));
  }

  /**
   * Changes the current limit of the arm motor.
   *
   * @param limit The limit, in amps.
   */
  public void currentLimit(double limit) {
    hardware.currentLimit(limit);
  }

  public Command climbSetup() {
    return goTo(Radians.of(0));
  }

  public Command climbExecute() {
    return goTo(Radians.of(0));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
