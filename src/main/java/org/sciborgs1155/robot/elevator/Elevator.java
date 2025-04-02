package org.sciborgs1155.robot.elevator;

import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;

import org.sciborgs1155.lib.Assertion;
import static org.sciborgs1155.lib.Assertion.eAssert;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Constants;
import static org.sciborgs1155.robot.Constants.TUNING;
import org.sciborgs1155.robot.Robot;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.BASE_FROM_CHASSIS;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.CARRIAGE_FROM_CHASSIS;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.HIGH_FIVE_DELAY;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_ACCEL;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_EXTENSION;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MIN_EXTENSION;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.POSITION_TOLERANCE;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.RAY_HIGH;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.RAY_LOW;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.RAY_MIDDLE;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kA;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kD;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kG;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kI;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kP;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kS;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kV;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

@Logged
public class Elevator extends SubsystemBase implements AutoCloseable {
  private final ElevatorIO hardware;

  private final SysIdRoutine sysIdRoutine;

  /**
   * @return Creates a Real or Sim elevator based on {@link Robot#isReal()}.
   */
  public static Elevator create() {
    return new Elevator(Robot.isReal() ? new RealElevator() : new SimElevator());
  }

  /**
   * Method to create a non-existent elevator.
   *
   * @return Non-real elevator object.
   */
  public static Elevator none() {
    return new Elevator(new NoElevator());
  }

  @Logged
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(MetersPerSecond), MAX_ACCEL.in(MetersPerSecondPerSecond)));

  @Logged private final ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

  @NotLogged
  private final ElevatorVisualizer setpoint =
      new ElevatorVisualizer("setpoint visualizer", new Color8Bit(0, 0, 255));

  @NotLogged
  private final ElevatorVisualizer measurement =
      new ElevatorVisualizer("measurement visualizer", new Color8Bit(255, 0, 0));

  @NotLogged private final DoubleEntry S = Tuning.entry("/Robot/tuning/elevator/kS", kS);
  @NotLogged private final DoubleEntry G = Tuning.entry("/Robot/tuning/elevator/kG", kG);
  @NotLogged private final DoubleEntry V = Tuning.entry("/Robot/tuning/elevator/kV", kV);
  @NotLogged private final DoubleEntry A = Tuning.entry("/Robot/tuning/elevator/kA", kA);

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
          "Robot/elevator/quasistatic forward",
          sysIdRoutine
              .quasistatic(Direction.kForward)
              .until(() -> atPosition(Level.L4.extension.in(Meters)))
              .withName("elevator quasistatic forward"));
      SmartDashboard.putData(
          "Robot/elevator/quasistatic backward",
          sysIdRoutine
              .quasistatic(Direction.kReverse)
              .until(() -> atPosition(MIN_EXTENSION.in(Meters) + 0.1))
              .withName("elevator quasistatic backward"));
      SmartDashboard.putData(
          "Robot/elevator/dynamic forward",
          sysIdRoutine
              .dynamic(Direction.kForward)
              .until(() -> atPosition(Level.L4.extension.in(Meters)))
              .withName("elevator dynamic forward"));
      SmartDashboard.putData(
          "Robot/elevator/dynamic backward",
          sysIdRoutine
              .dynamic(Direction.kReverse)
              .until(() -> atPosition(MIN_EXTENSION.in(Meters) + 0.1))
              .withName("elevator dynamic backward"));
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

  public Command homingSequence() {
    return run(() -> hardware.setVoltage(-0.5)).until(() -> hardware.velocity() < 0.005).andThen(() -> hardware.resetPosition());
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
  @Logged
  public double position() {
    return hardware.position();
  }

  /**
   * @return Velocity of the elevator in meters per second.
   */
  @Logged
  public double velocity() {
    return hardware.velocity();
  }

  /**
   * @return Desired position of the elevator in meters
   */
  @Logged
  public double positionSetpoint() {
    return pid.getSetpoint().position;
  }

  /**
   * @return Desired velocity of the elevator in meters per second.
   */
  @Logged
  public double velocitySetpoint() {
    return pid.getSetpoint().velocity;
  }

  /**
   * @return Whether or not the elevator is at its desired state.
   */
  @Logged
  public boolean atGoal() {
    return pid.atGoal();
  }

  @Logged
  public Pose3d stage1() {
    return new Pose3d(
        BASE_FROM_CHASSIS.plus(
            new Translation3d(
                Math.sin(7 * Math.PI / 180) * hardware.position() / 2, 0, hardware.position() / 2)),
        Rotation3d.kZero);
  }

  @Logged
  public Pose3d carriage() {
    return new Pose3d(
        CARRIAGE_FROM_CHASSIS.plus(
            new Translation3d(
                Math.sin(7 * Math.PI / 180) * hardware.position(), 0, hardware.position())),
        Rotation3d.kZero);
  }

  /**
   * Method to set voltages to the hardware based off feedforward and feedback. Control should only
   * be used in a command.
   *
   * @param position Goal height for the elevator to achieve.
   */
  private void update(double position) {
    double goal =
        Double.isNaN(position)
            ? MIN_EXTENSION.in(Meters)
            : MathUtil.clamp(position, MIN_EXTENSION.in(Meters), MAX_EXTENSION.in(Meters));
    double lastVelocity = pid.getSetpoint().velocity;
    double feedback = pid.calculate(hardware.position(), goal);
    double feedforward = ff.calculateWithVelocities(lastVelocity, pid.getSetpoint().velocity);

    Epilogue.getConfig().backend.log("/Robot/elevator/elevator voltage", feedback + feedforward);
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

    if (TUNING) {
      ff.setKs(S.get());
      ff.setKg(G.get());
      ff.setKv(V.get());
      ff.setKa(A.get());
    }
    Epilogue.getConfig()
        .backend
        .log(
            "/Robot/elevator/command",
            Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
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
