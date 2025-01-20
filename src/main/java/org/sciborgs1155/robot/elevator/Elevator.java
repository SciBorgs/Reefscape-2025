package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.lib.Assertion.eAssert;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
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

    pid.setTolerance(POSITION_TOLERANCE.in(Meters));
    pid.reset(hardware.position());
    pid.setGoal(MIN_HEIGHT.in(Meters));

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

  /**
   * drives elevator to its minimum
   *
   * @return a command which drives the elevator to its minimum height
   */
  public Command retract() {
    return run(() -> update(MIN_HEIGHT.in(Meters)));
  }

  /**
   * drives elevator to one of the 4 levels
   *
   * @param level an enum that can be L1-L4
   * @return a command which drives the elevator to one of the 4 levels
   */
  public Command scoreLevel(Level level) {
    return goTo(level.getHeight().in(Meters));
  }

  /**
   * drives elevator to the desired height, within its physical boundaries
   *
   * @param height desired height in meters
   * @return a command which drives the elevator to the desired height
   */
  public Command goTo(double height) {
    return run(() -> update(height));
  }

  /**
   * give measured encoder height of elevator
   *
   * @return position of the elevator in meters
   */
  @Log.NT
  public double position() {
    return hardware.position();
  }

  /**
   * give measured encoder velocity of elevator
   *
   * @return velocity of the elevator in meters per second
   */
  @Log.NT
  public double velocity() {
    return hardware.velocity();
  }

  /**
   * give desired encoder height of elevator
   *
   * @return desired position of the elevator in meters
   */
  @Log.NT
  public double positionSetpoint() {
    return pid.getSetpoint().position;
  }

  /**
   * give desired encoder velocity of elevator
   *
   * @return desired velocity of the elevator in meters per second
   */
  @Log.NT
  public double velocitySetpoint() {
    return pid.getSetpoint().velocity;
  }

  /**
   * @return whether or not the elevator is at its desired state
   */
  public boolean atGoal() {
    return pid.atGoal();
  }

  /**
   * method to set voltages to the hardware based off feedforward and feedback control should only
   * be used in a command
   *
   * @param position goal height for the elevator to achieve
   */
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

  /**
   * drives the elevator to a desired height and asserts that it has reached said height can be used
   * in unit tests or for real-life systems checks
   *
   * @param testHeight desired height for the elevator to reach
   * @return command to drive elevator to desired position and check whether it has achieved its
   *     goal
   */
  public Test goToTest(Distance testHeight) {
    Command testCommand = goTo(testHeight.in(Meters)).until(this::atGoal).withTimeout(3);
    Set<Assertion> assertions =
        Set.of(
            eAssert(
                "Elevator syst check (position)",
                () -> testHeight.in(Meters),
                this::position,
                POSITION_TOLERANCE.in(Meters)));

    return new Test(testCommand, assertions);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
