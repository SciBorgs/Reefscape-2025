package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.lib.Assertion.eAssert;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Set;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.Constants.Field.Level;
import org.sciborgs1155.robot.Robot;

public class Elevator extends SubsystemBase implements Logged, AutoCloseable {
  public static Elevator create() {
    return Robot.isReal() ? new Elevator(new RealElevator()) : new Elevator(new SimElevator());
  }

  public static Elevator none() {
    return new Elevator(new NoElevator());
  }

  private final ElevatorIO hardware;

  private final SysIdRoutine sysIdRoutine;

  @Log.NT
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(MetersPerSecond), MAX_ACCEL.in(MetersPerSecondPerSecond)));

  @Log.NT private final ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

  @Log.NT
  private final ElevatorVisualizer setpoint = new ElevatorVisualizer(new Color8Bit(0, 0, 255));

  @Log.NT
  private final ElevatorVisualizer measurement = new ElevatorVisualizer(new Color8Bit(255, 0, 0));

  public Elevator(ElevatorIO hardware) {
    this.hardware = hardware;
    setDefaultCommand(retract());
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(v -> hardware.setVoltage(v.in(Volts)), null, this));

    SmartDashboard.putData(
        "pivot quasistatic forward", sysIdRoutine.quasistatic(Direction.kForward));
    SmartDashboard.putData(
        "pivot quasistatic backward", sysIdRoutine.quasistatic(Direction.kReverse));
    SmartDashboard.putData("pivot dynamic forward", sysIdRoutine.dynamic(Direction.kForward));
    SmartDashboard.putData("pivot dynamic backward", sysIdRoutine.dynamic(Direction.kReverse));
  }

  public Command extend() {
    return run(() -> update(MAX_HEIGHT.in(Meters)));
  }

  public Command retract() {
    return run(() -> update(MIN_HEIGHT.in(Meters)));
  }

  public Command scoreLevel(Level level) {
    return run(() -> update(level.getHeight()));
  }

  /**
   * @param height in meters
   * @return
   */
  public Command goTo(double height) {
    return run(() -> update(height));
  }

  @Log.NT
  public double position() {
    return hardware.position();
  }

  @Log.NT
  public double velocity() {
    return hardware.velocity();
  }

  @Log.NT
  public double positionSetpoint() {
    return pid.getSetpoint().position;
  }

  @Log.NT
  public double velocitySetpoint() {
    return pid.getSetpoint().velocity;
  }

  private void update(double position) {
    position = MathUtil.clamp(position, MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters));

    double lastVelocity = pid.getSetpoint().velocity;
    double feedback = pid.calculate(hardware.position(), position);
    double feedforward = ff.calculateWithVelocities(lastVelocity, pid.getSetpoint().velocity);

    hardware.setVoltage(feedforward + feedback);
  }

  @Override
  public void periodic() {
    setpoint.setLength(positionSetpoint());
    measurement.setLength(position());
  }

  public Test systemsCheck() {
    Command testCommand = goTo(TEST_HEIGHT.in(Meters));
    Set<Assertion> assertions =
        Set.of(
            eAssert(
                "Elevator syst check (position)",
                () -> TEST_HEIGHT.in(Meters),
                this::position,
                .1));

    return new Test(testCommand, assertions);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
