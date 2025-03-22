package org.sciborgs1155.robot.scoral;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.lib.Assertion.tAssert;
import static org.sciborgs1155.robot.Ports.Scoral.*;
import static org.sciborgs1155.robot.scoral.ScoralConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.Set;
import monologue.Logged;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Beambreak;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

public class Scoral extends SubsystemBase implements Logged, AutoCloseable {
  private final SimpleMotor motor;

  public final Beambreak beambreak;
  public final Trigger blocked;

  /** Creates a Scoral based on if it is utilizing hardware. */
  public static Scoral create() {
    return Robot.isReal() ? new Scoral(realMotor(), Beambreak.real(BEAMBREAK)) : none();
  }

  /** Creates a Scoral sans hardware or simulation. */
  public static Scoral none() {
    return new Scoral(SimpleMotor.none(), Beambreak.none());
  }

  /** Creates a SimpleMotor with the appropriate configurations for a Scoral with hardware. */
  private static SimpleMotor realMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    return SimpleMotor.talon(new TalonFX(ROLLER, "rio"), config);
  }

  public Scoral(SimpleMotor motor, Beambreak beambreak) {
    this.motor = motor;
    this.beambreak = beambreak;
    this.blocked = new Trigger(beambreak::get); // it spontaneously negated....

    setDefaultCommand(stop());
  }

  /** Runs the motor to move a coral out of the scoral outwards. */
  public Command score() {
    return run(() -> motor.set(SCORE_POWER)).withName("score");
  }

  public Command scoreSlow() {
    return run(() -> motor.set(SCORE_POWER / 2)).withName("score slow");
  }

  public Command algae() {
    return run(() -> motor.set(-SCORE_POWER)).withName("algaeing");
  }

  public Command intake() {
    return run(() -> motor.set(INTAKE_POWER * 0.6)).withName("intake");
  }

  /** Stops the motor */
  public Command stop() {
    return run(() -> motor.set(0)).withName("stop");
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
    motor.close();
  }
}
