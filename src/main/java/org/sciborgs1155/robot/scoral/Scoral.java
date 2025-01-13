package org.sciborgs1155.robot.scoral;

import static org.sciborgs1155.robot.scoral.ScoralConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Scoral extends SubsystemBase implements Logged, AutoCloseable {

  @Log.NT private final ScoralIO hardware;

  public Scoral(ScoralIO hardware) {
    this.hardware = hardware;
  }

  public static Scoral create() {
    return Robot.isReal() ? new Scoral(new RealScoral()) : new Scoral(new SimScoral());
  }

  public static Scoral none() {
    return new Scoral(new NoScoral());
  }

  /** Runs the motor to outtake, as in pushing out, a coral. */
  public Command outtake() {
    return run(() -> hardware.setPower(POWER)).withName("outtake");
  }

  /** Runs the motor to intake, as in pulling in, a coral. */
  public Command intake() {
    return run(() -> hardware.setPower(-POWER)).withName("intake");
  }

  /** Returns the value of the beambreak. */
  @Log.NT
  public boolean beambreak() {
    return hardware.beambreak();
  }

  @Override
  public void periodic() {
    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
