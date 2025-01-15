package org.sciborgs1155.robot.scoral;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Scoral.*;
import static org.sciborgs1155.robot.scoral.ScoralConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Robot;

public class Scoral extends SubsystemBase implements Logged, AutoCloseable {
  private final SimpleMotor hardware;

  private final DigitalInput beambreak = new DigitalInput(BEAMBREAK);

  private static SimpleMotor realMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    return SimpleMotor.talon(new TalonFX(ROLLER), config);
  }

  public Scoral(SimpleMotor hardware) {
    this.hardware = hardware;
  }

  public static Scoral create() {
    return new Scoral(Robot.isReal() ? realMotor() : SimpleMotor.none());
  }

  public static Scoral none() {
    return new Scoral(SimpleMotor.none());
  }

  /** Runs the motor to outtake, as in pushing out, a coral. */
  public Command outtake() {
    return run(() -> hardware.set(POWER)).withName("outtake");
  }

  /** Runs the motor to intake, as in pulling in, a coral. */
  public Command intake() {
    return run(() -> hardware.set(-POWER)).withName("intake");
  }

  /** Returns the value of the beambreak. */
  @Log.NT
  public boolean beambreak() {
    return beambreak.get();
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
