package org.sciborgs1155.robot.coroller;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.coroller.CorollerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Ports.GroundIntake;
import org.sciborgs1155.robot.Robot;

/** Simple roller subsystem used for intaking/outtaking coral. */
public class Coroller extends SubsystemBase implements Logged, AutoCloseable {
  /** Interface for interacting with the motor itself. */
  private final SimpleMotor hardware;

  /**
   * Returns a new {@link Coroller} subsystem, which will have real hardware if the robot is real,
   * and no hardware connection if it isn't.
   */
  public static Coroller create() {
    return new Coroller(Robot.isReal() ? real() : SimpleMotor.none());
  }

  /** Creates a new {@link Coroller} with no hardware interface(does nothing). */
  public static Coroller none() {
    return new Coroller(SimpleMotor.none());
  }

  private Coroller(SimpleMotor hardware) {
    this.hardware = hardware;
    setDefaultCommand(stop());
  }

  private static SimpleMotor real() {
    /** Configuration of the motor used in the Coroller. */
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = STRATOR_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return SimpleMotor.talon(new TalonFX(GroundIntake.ROLLER_MOTOR), config);
  }

  /** Makes the roller spin inwards(towards robot). */
  public Command intake() {
    return run(() -> hardware.set(INTAKE_POWER)).withName("Intaking");
  }

  /** Makes the roller spin outwards(away from robot). */
  public Command outtake() {
    return run(() -> hardware.set(OUTTAKE_POWER)).withName("Outtaking");
  }

  /** Stops the roller motors. */
  public Command stop() {
    return run(() -> hardware.set(0)).withName("Stopping");
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
