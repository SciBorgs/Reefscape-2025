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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
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
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;

public class Elevator extends SubsystemBase implements Logged, AutoCloseable {
  private final ElevatorIO hardware;

  private final SysIdRoutine sysIdRoutine;
  private double lastGoal = MIN_EXTENSION.in(Meters);

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

  @Log.NT private final ElevatorVisualizer goal = new ElevatorVisualizer(new Color8Bit(0, 0, 255));

  @Log.NT
  private final ElevatorVisualizer measurement = new ElevatorVisualizer(new Color8Bit(255, 0, 0));

  private final DoubleEntry S = Tuning.entry("/Robot/tuning/elevator/kS", kS);
  private final DoubleEntry G = Tuning.entry("/Robot/tuning/elevator/kG", kG);
  private final DoubleEntry V = Tuning.entry("/Robot/tuning/elevator/kV", kV);
  private final DoubleEntry A = Tuning.entry("/Robot/tuning/elevator/kA", kA);

  public Elevator(ElevatorIO hardware) {
    setDefaultCommand(retract());

    this.hardware = hardware;

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
          sysIdRoutine
              .quasistatic(Direction.kForward)
              .until(() -> atPosition(Level.L4.extension.in(Meters)))
              .withName("elevator quasistatic forward"));
      SmartDashboard.putData(
          "elevator quasistatic backward",
          sysIdRoutine
              .quasistatic(Direction.kReverse)
              .until(() -> atPosition(MIN_EXTENSION.in(Meters) + 0.1))
              .withName("elevator quasistatic backward"));
      SmartDashboard.putData(
          "elevator dynamic forward",
          sysIdRoutine
              .dynamic(Direction.kForward)
              .until(() -> atPosition(Level.L4.extension.in(Meters)))
              .withName("elevator dynamic forward"));
      SmartDashboard.putData(
          "elevator dynamic backward",
          sysIdRoutine
              .dynamic(Direction.kReverse)
              .until(() -> atPosition(MIN_EXTENSION.in(Meters) + 0.1))
              .withName("elevator dynamic backward"));

      Tuning.changes(S).onTrue(runOnce(() -> hardware.setS(S.get())).asProxy());
      Tuning.changes(V).onTrue(runOnce(() -> hardware.setV(V.get())).asProxy());
      Tuning.changes(A).onTrue(runOnce(() -> hardware.setA(A.get())).asProxy());
      Tuning.changes(G).onTrue(runOnce(() -> hardware.setG(G.get())).asProxy());
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
            .add(() -> lastGoal))
        .withName("manual elevator");
  }

  /**
   * Drives elevator to the desired height, within its physical boundaries.
   *
   * @param height Desired height in meters.
   * @return A command which drives the elevator to the desired height.
   */
  public Command goTo(DoubleSupplier height) {
    lastGoal = height.getAsDouble();
    return run(() -> hardware.setGoal(lastGoal)).finallyDo(() -> hardware.setVoltage(0));
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
   * @return Whether or not the elevator is at its desired state.
   */
  @Log.NT
  public boolean atGoal() {
    return atPosition(lastGoal);
  }

  @Log.NT
  public Pose3d stage1() {
    return new Pose3d(
        BASE_FROM_CHASSIS.plus(
            new Translation3d(
                Math.sin(7 * Math.PI / 180) * hardware.position() / 2, 0, hardware.position() / 2)),
        Rotation3d.kZero);
  }

  @Log.NT
  public Pose3d carriage() {
    return new Pose3d(
        CARRIAGE_FROM_CHASSIS.plus(
            new Translation3d(
                Math.sin(7 * Math.PI / 180) * hardware.position(), 0, hardware.position())),
        Rotation3d.kZero);
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
    goal.setLength(lastGoal);
    measurement.setLength(position());

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
