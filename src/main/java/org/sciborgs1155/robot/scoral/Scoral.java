package org.sciborgs1155.robot.scoral;

import static org.sciborgs1155.lib.Assertion.tAssert;
import static org.sciborgs1155.robot.scoral.ScoralConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.Set;
import monologue.Logged;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;

public class Scoral extends SubsystemBase implements Logged, AutoCloseable {
  public final Trigger blocked;
  private final ScoralIO hardware;

  /** Creates a Scoral based on if it is utilizing hardware. */
  public static Scoral create() {
    return new Scoral(Robot.isReal() ? new RealScoral() : new SimCoral());
  }

  /** Creates a Scoral sans hardware or simulation. */
  public static Scoral none() {
    return new Scoral(new NoScoral());
  }

  public Scoral(ScoralIO hardware) {
    this.hardware = hardware;

    this.blocked =
        new Trigger(
            Robot.isReal()
                ? () -> !hardware.hasCoral()
                : () -> hardware.hasCoral()); // it spontaneously negated.... in real life

    setDefaultCommand(stop());
  }

  /** Runs the motor to move a coral out of the scoral outwards. */
  public Command score() {
    return run(() -> hardware.set(SCORE_POWER, true)).withName("score");
  }

  public Command score(Level level) {
    return run(() -> hardware.set(level == Level.L4 ? SCORE_POWER : 0.6 * SCORE_POWER, true))
        .withName("score level");
  }

  public Command scoreSlow() {
    return run(() -> hardware.set(SCORE_POWER / 5, true)).withName("score");
  }

  public Command algae() {
    return run(() -> hardware.set(-SCORE_POWER, false)).withName("algaeing");
  }

  public Command intake() {
    return run(() -> hardware.set(INTAKE_POWER * 0.6, false)).withName("intake");
  }

  /** Stops the motor */
  public Command stop() {
    return run(() -> hardware.set(0, false)).withName("stop");
  }

  @Override
  public void periodic() {
    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }

  /**
   * Sets the scoral to intake, expecting a coral. Can also be used to test beambreak.
   *
   * @return Command to set the scoral to intake and check whether it has a coral.
   */
  public Test intakeTest() {
    Command testCommand = intake().until(blocked).withTimeout(5);
    Set<Assertion> assertions =
        Set.of(
            tAssert(blocked, "Scoral syst check (beambreak broken)", () -> "broken: " + blocked));
    return new Test(testCommand, assertions);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
