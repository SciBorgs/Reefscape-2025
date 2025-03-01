package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.lib.Assertion.eAssert;
import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.BetterElevatorFeedforward;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

public class Elevator extends SubsystemBase implements Logged, AutoCloseable {
  private final ElevatorIO hardware;

  private final SysIdRoutine sysIdRoutine;

  /**
   * @return Creates a Real or Sim elevator based on {@link Robot#isReal()}.
   */
  public static Elevator create() {
    return new Elevator(Robot.isReal() ? new RealElevator() : new SimElevator());
  }

  /**
   * Method to create a no elevator.
   *
   * @return No elevator object.
   */
  public static Elevator none() {
    return new Elevator(new NoElevator());
  }

  @Log.NT
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(MetersPerSecond), MAX_ACCEL.in(MetersPerSecondPerSecond)));

  @Log.NT
  private final BetterElevatorFeedforward ff = new BetterElevatorFeedforward(kS, kG, kV, kA);

  @Log.NT
  private final ElevatorVisualizer setpoint = new ElevatorVisualizer(new Color8Bit(0, 0, 255));

  @Log.NT
  private final ElevatorVisualizer measurement = new ElevatorVisualizer(new Color8Bit(255, 0, 0));

  private final DoubleEntry S = Tuning.entry("/Robot/elevator/kS", kS);
  private final DoubleEntry G = Tuning.entry("/Robot/elevator/kG", kG);
  private final DoubleEntry V = Tuning.entry("/Robot/elevator/kV", kV);
  private final DoubleEntry A = Tuning.entry("/Robot/elevator/kA", kA);

  public Elevator(ElevatorIO hardware) {
    setDefaultCommand(retract());

    this.hardware = hardware;

    pid.setTolerance(POSITION_TOLERANCE.in(Meters));
    pid.reset(hardware.position());
    pid.setGoal(MIN_EXTENSION.in(Meters));

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(2),
                null,
                (state) -> SignalLogger.writeString("elevator state", state.toString())),
            new SysIdRoutine.Mechanism(v -> hardware.setVoltage(v.in(Volts)), null, this));

    if (TUNING) {
      SmartDashboard.putData(
          "elevator quasistatic forward",
          sysIdRoutine.quasistatic(Direction.kForward).withName("elevator quasistatic forward"));
      SmartDashboard.putData(
          "elevator quasistatic backward",
          sysIdRoutine.quasistatic(Direction.kReverse).withName("elevator quasistatic backward"));
      SmartDashboard.putData(
          "elevator dynamic forward",
          sysIdRoutine.dynamic(Direction.kForward).withName("elevator dynamic forward"));
      SmartDashboard.putData(
          "elevator dynamic backward",
          sysIdRoutine.dynamic(Direction.kReverse).withName("elevator dynamic backward"));
    }
  }

  /**
   * Drives elevator to its minimum.
   *
   * @return A command which drives the elevator to its minimum height.
   */
  public Command retract() {
    return goTo(MIN_EXTENSION.in(Meters)).withName("retracting");
  }

  /**
   * Drives elevator to one of the 4 levels.
   *
   * @param level An enum that can be L1-L4.
   * @return A command which drives the elevator to one of the 4 levels.
   */
  public Command scoreLevel(Level level) {
    return goTo(level.extension.in(Meters)).withName("scoring");
  }

  /**
   * Goes to an offset height above the level given in order to clean algae; ONLY L2 and L3!
   *
   * @param level An enum that should be either L2 or L3
   */
  public Command clean(Level level) {
    if (level == Level.L1 || level == Level.L4) {
      FaultLogger.report(
          "Algae level",
          "An invalid level (L1, L4) has been passed to the clean command",
          FaultType.WARNING);
      return retract();
    }

    return goTo(level.extension.in(Meters)).withName("cleaning");
  }

  public Command manualElevator(InputStream input) {
    return goTo(input
            .deadband(.15, 1)
            .scale(MAX_VELOCITY.in(MetersPerSecond))
            .scale(2)
            .scale(Constants.PERIOD.in(Seconds))
            .rateLimit(MAX_ACCEL.in(MetersPerSecondPerSecond))
            .add(() -> pid.getGoal().position))
        .withName("manual elevator");
  }

  /**
   * Drives elevator to the desired height, within its physical boundaries.
   *
   * @param height Desired height in meters.
   * @return A command which drives the elevator to the desired height.
   */
  public Command goTo(DoubleSupplier height) {
    return run(() -> update(height.getAsDouble())).finallyDo(() -> hardware.setVoltage(0));
  }

  public Command goTo(double height) {
    return goTo(() -> height);
  }

  /**
   * @return A very safe and serious command....
   */
  public Command highFive() {
    return goTo(() -> RAY_HIGH.in(Meters))
        .until(() -> atGoal())
        .andThen(Commands.waitSeconds(HIGH_FIVE_DELAY.in(Seconds)))
        .andThen(goTo(() -> RAY_LOW.in(Meters)).until(() -> atGoal()))
        .andThen(Commands.waitSeconds(HIGH_FIVE_DELAY.in(Seconds)))
        .andThen(goTo(RAY_MIDDLE.in(Meters)));
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
  @Log.NT
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
    position =
        Double.isNaN(position)
            ? MIN_EXTENSION.in(Meters)
            : MathUtil.clamp(position, MIN_EXTENSION.in(Meters), MAX_EXTENSION.in(Meters));
    double lastVelocity = pid.getSetpoint().velocity;
    double feedback = pid.calculate(hardware.position(), position);
    double feedforward = ff.calculateWithVelocities(lastVelocity, pid.getSetpoint().velocity);

    hardware.setVoltage(feedforward + feedback);
  }

  public boolean atPosition(double position) {
    return Meters.of(position).minus(Meters.of(position())).magnitude()
        < POSITION_TOLERANCE.in(Meters);
  }

  @Override
  /**
   * This method will be called periodically and is used to update the setpoint and measurement
   * lengths.
   */
  public void periodic() {
    setpoint.setLength(positionSetpoint());
    measurement.setLength(position());

    ff.setKs(S.get());
    ff.setKg(G.get());
    ff.setKv(V.get());
    ff.setKa(A.get());

    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
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
    Command testCommand = goTo(testHeight.in(Meters)).until(this::atGoal).withTimeout(5);
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
