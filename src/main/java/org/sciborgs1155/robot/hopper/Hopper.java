package org.sciborgs1155.robot.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.sciborgs1155.robot.Robot;

public class Hopper extends SubsystemBase implements AutoCloseable {
  public static Hopper create() {
    return Robot.isReal() ? new Hopper(new RealHopper()) : Hopper.none();
  }

  public static Hopper none() {
    return new Hopper(new NoHopper());
  }

  public final HopperIO hardware;
  public final Trigger beambreakTrigger;

  public Hopper(HopperIO hardware) {
    this.hardware = hardware;
    this.beambreakTrigger = new Trigger(hardware::beambreak);
  }

  public Command run(double power) {
    return runOnce(() -> hardware.setPower(power));
  }

  public Command intake() {
    return run(HopperConstants.INTAKE_POWER); // more logic later
  }

  public Command outtake() {
    return run(-HopperConstants.INTAKE_POWER);
  }

  public Command stop() {
    return run(0);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
