package org.sciborgs1155.robot.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.sciborgs1155.robot.Robot;

public class Hopper extends SubsystemBase implements AutoCloseable {
  /** Creates a real or non-existent hopper based on {@link Robot#isReal()}. */
  public static Hopper create() {
    return Robot.isReal() ? new Hopper(new RealHopper()) : Hopper.none();
  }

  /** Creates a non-existent hopper. */
  public static Hopper none() {
    return new Hopper(new NoHopper());
  }

  private final HopperIO hardware;
  public final Trigger beambreakTrigger;

  public Hopper(HopperIO hardware) {
    this.hardware = hardware;
    this.beambreakTrigger = new Trigger(hardware::beambreak);
  }

  /**
   * Runs the motors of the hopper.
   *
   * @param power The power to set the hoppers motors to.
   * @return A command to set the power of the hopper motors.
   */
  public Command run(double power) {
    return runOnce(() -> hardware.setPower(power));
  }

  /**
   * Intakes corals from human player.
   *
   * @return A command to intake corals.
   */
  public Command intake() {
    return run(HopperConstants.INTAKE_POWER); // more logic later
  }

  /**
   * Outtake corals.
   *
   * @return A command to outtake corals.
   */
  public Command outtake() {
    return run(-HopperConstants.INTAKE_POWER);
  }

  /**
   * Stops the hopper.
   *
   * @return A command to stop the hopper.
   */
  public Command stop() {
    return run(0);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
