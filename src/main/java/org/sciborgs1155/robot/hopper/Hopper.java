package org.sciborgs1155.robot.hopper;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Hopper.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.sciborgs1155.lib.Beambreak;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Robot;

public class Hopper extends SubsystemBase implements AutoCloseable {
  private final SimpleMotor motor;
  private final Beambreak beambreak;
  public final Trigger beambreakTrigger;

  /** Creates a Hopper based on whether it is utilizing hardware. */
  public static Hopper create() {
    return Robot.isReal() ? new Hopper(realMotor(), Beambreak.real(BEAMBREAK)) : none();
  }

  /** Creates a hopper sans hardware or simulation. */
  public static Hopper none() {
    return new Hopper(SimpleMotor.none(), Beambreak.none());
  }

  /** Generates a simple motor with the appropriate configurations as real hardware. */
  private static SimpleMotor realMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = HopperConstants.CURRENT_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    return SimpleMotor.talon(new TalonFX(MOTOR), config);
  }

  public Hopper(SimpleMotor motor, Beambreak beambreak) {
    this.motor = motor;
    this.beambreak = beambreak;
    this.beambreakTrigger = new Trigger(beambreak::get);

    setDefaultCommand(stop());
  }

  /**
   * Runs the motors of the hopper.
   *
   * @param power The power to set the hoppers motors to.
   * @return A command to set the power of the hopper motors.
   */
  public Command run(double power) {
    return runOnce(() -> motor.set(power));
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
    motor.close();
    beambreak.close();
  }
}
