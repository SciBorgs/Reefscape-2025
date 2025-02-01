package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.lib.Assertion.eAssert;
import static org.sciborgs1155.robot.Constants.Field.algaeOffset;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
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
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.Constants.Field.Level;
import org.sciborgs1155.robot.Robot;

public class Elevator extends SubsystemBase implements Logged, AutoCloseable {
  public static Elevator create() {
    return new Elevator(Robot.isReal() ? new RealElevator() : new SimElevator());
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
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("elevator state", state.toString())),
            new SysIdRoutine.Mechanism(v -> hardware.setVoltage(v.in(Volts)), null, this));

    SmartDashboard.putData(
        "pivot quasistatic forward", sysIdRoutine.quasistatic(Direction.kForward));
    SmartDashboard.putData(
        "pivot quasistatic backward", sysIdRoutine.quasistatic(Direction.kReverse));
    SmartDashboard.putData("pivot dynamic forward", sysIdRoutine.dynamic(Direction.kForward));
    SmartDashboard.putData("pivot dynamic backward", sysIdRoutine.dynamic(Direction.kReverse));
  }

  /**
   * Drives elevator to its minimum.
   *
   * @return A command which drives the elevator to its minimum height.
   */
  public Command retract() {
    return goTo(MIN_HEIGHT.in(Meters));
  }

  /**
   * Drives elevator to one of the 4 levels.
   *
   * @param level An enum that can be L1-L4.
   * @return A command which drives the elevator to one of the 4 levels.
   */
  public Command scoreLevel(Level level) {
    return goTo(level.height.in(Meters));
  }

  /**
   * Goes to an offset height above the level given in order to clean algae; ONLY L2 and L3!
   *
   * @param level An enum that should be either L2 or L3
   */
  public Command clean(Level level) {
    if (level == Level.L1 || level == Level.L4) {
      retract();
      FaultLogger.report(
          "Algae level",
          "An invalid level has been passed to the clean command; L1 or L4",
          FaultType.WARNING);
    }

    return goTo(level.height.plus(algaeOffset).in(Meters));
  }

  /**
   * Drives elevator to the desired height, within its physical boundaries.
   *
   * @param height Desired height in meters.
   * @return A command which drives the elevator to the desired height.
   */
  public Command goTo(double height) {
    return run(() -> update(height));
  }

  /**
   * @return Position of the elevator in meters.
   */
  @Log.NT
  public double position() {
    return hardware.position();
  }

  /**
   * @return Velocity of the elevator in meters per second.
   */
  @Log.NT
  public double velocity() {
    return hardware.velocity();
  }

  /**
   * @return Desired position of the elevator in meters
   */
  @Log.NT
  public double positionSetpoint() {
    return pid.getSetpoint().position;
  }

  /**
   * @return Desired velocity of the elevator in meters per second.
   */
  @Log.NT
  public double velocitySetpoint() {
    return pid.getSetpoint().velocity;
  }

  /**
   * @return Whether or not the elevator is at its desired state.
   */
  public boolean atGoal() {
    return pid.atGoal();
  }

  /**
   * Method to set voltages to the hardware based off feedforward and feedback. Control should only
   * be used in a command.
   *
   * @param position Goal height for the elevator to achieve.
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
   * Drives the elevator to a desired height and asserts that it has reached said height. Can be
   * used in unit tests or for real-life systems checks.
   *
   * @param testHeight Desired height for the elevator to reach.
   * @return Command to drive elevator to desired position and check whether it has achieved its
   *     goal.
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
