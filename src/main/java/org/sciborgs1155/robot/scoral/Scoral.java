package org.sciborgs1155.robot.scoral;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Scoral.*;
import static org.sciborgs1155.robot.scoral.ScoralConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.Beambreak;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Robot;

public class Scoral extends SubsystemBase implements Logged, AutoCloseable {
  private final SimpleMotor hardware;

  private final Beambreak beambreak;
  public final Trigger beambreakTrigger;

  /**
   * creates a Scoral based on if it is simulated or real hardware
   */
  public static Scoral create() {
    return Robot.isReal() ? new Scoral(realMotor(), Beambreak.real(BEAMBREAK)) : none();
  }

  /**
   * creates a Scoral without hardware or simulation
   */
  public static Scoral none() {
    return new Scoral(SimpleMotor.none(), Beambreak.none());
  }

  private static SimpleMotor realMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    return SimpleMotor.talon(new TalonFX(ROLLER), config);
  }

  public Scoral(SimpleMotor hardware, Beambreak beambreak) {
    this.hardware = hardware;
    this.beambreak = beambreak;
    beambreakTrigger = new Trigger(beambreak::get);
  }

  /** Runs the motor to move a coral out of the scoral outwards. */
  public Command outtake() {
    return run(() -> hardware.set(POWER)).withName("outtake");
  }

  /** Runs the motor to move a coral into the scoral. */
  public Command intake() {
    return run(() -> hardware.set(POWER)).withName("intake");
  }

  /** Stops the motor */
  public Command stop() {
    return run(() -> hardware.set(0)).withName("stop");
  }

  /** Returns the value of the beambreak, which is false when the beam is broken. */
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
