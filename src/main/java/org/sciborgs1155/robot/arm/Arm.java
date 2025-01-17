package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.Set;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Assertion.EqualityAssertion;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.Robot;

/** Simple Arm subsystem used for climbing and intaking coral from the ground. */
public class Arm extends SubsystemBase implements Logged, AutoCloseable {
  /** Interface for interacting with the motor itself. */
  @Log.NT private final ArmIO hardware;

  /** Trapezoid profile feedback (PID) controller */
  @Log.NT
  private final ProfiledPIDController fb =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCEL.in(RadiansPerSecondPerSecond)));

  /** Arm feed forward controller. */
  private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

  /** Routine for recording and analyzing motor data. */
  private final SysIdRoutine sysIdRoutine;

  /** Arm visualization software. */
  @Log.NT private final Mechanism2d armCanvas = new Mechanism2d(60, 60);

  private final MechanismRoot2d armRoot = armCanvas.getRoot("ArmPivot", 30, 30);

  /** Arm visualizer. */
  private final MechanismLigament2d armLigament =
      armRoot.append(
          new MechanismLigament2d(
              "Arm",
              ARM_LENGTH.in(Centimeters),
              STARTING_ANGLE.in(Degrees),
              10,
              new Color8Bit(Color.kSkyBlue)));

  /**
   * Returns a new {@link Arm} subsystem, which will have real hardware if the robot is real, and
   * simulated if it isn't.
   */
  public static Arm create() {
    return new Arm(Robot.isReal() ? new RealArm() : new SimArm());
  }

  /** Creates a new {@link Arm} with no hardware interface(does nothing). */
  public static Arm none() {
    return new Arm(new NoArm());
  }

  /**
   * Constructor.
   *
   * @param hardware : The ArmIO object (real/simulated/nonexistent) that will be operated on.
   */
  private Arm(ArmIO hardware) {
    this.hardware = hardware;
    fb.setTolerance(POSITION_TOLERANCE.in(Radians));
    fb.reset(STARTING_ANGLE.in(Radians));
    fb.setGoal(STARTING_ANGLE.in(Radians));

    this.sysIdRoutine =
        new SysIdRoutine(
            new Config(Volts.of(0.5).per(Second), Volts.of(0.2), Seconds.of(5)),
            new Mechanism(voltage -> hardware.setVoltage(voltage.in(Volts)), null, this));
  }

  /**
   * @return The position in radians.
   */
  @Log.NT
  public double position() {
    return hardware.position();
  }

  /**
   * @return The position in radians/sec.
   */
  @Log.NT
  public double velocity() {
    return hardware.velocity();
  }

  @Log.NT
  /** Moves the arm towards a specified goal angle. */
  public Command goTo(Angle goal) {
    return run(() -> {
          double feedforward = ff.calculate(fb.getSetpoint().position, 0);
          double feedback = fb.calculate(hardware.position(), goal.in(Radians));
          hardware.setVoltage(feedback + feedforward);
        })
        .withName("Moving Arm To: " + goal.toString() + " radians");
  }

  /**
   * Changes the current limit of the arm motor.
   *
   * @param limit The limit, in amps.
   */
  public void currentLimit(double limit) {
    hardware.currentLimit(limit);
  }

  /**
   * A Test which moves the arm towards a goal angle and then asserts that it got there.
   *
   * @param goal The goal angle to which the arm will move to.
   * @return A Test object which moves the arm and checks it got to its destination.
   */
  public Test goToTest(Angle goal) {
    Command testCommand = goTo(goal).until(fb::atGoal).withTimeout(3).withName("Arm Test");
    EqualityAssertion atGoal =
        Assertion.eAssert(
            "arm angle", () -> goal.in(Radians), this::position, POSITION_TOLERANCE.in(Radians));
    return new Test(testCommand, Set.of(atGoal));
  }

  /**
   * Runs all 4 {@link SysIdRoutine}'s in sequence.
   *
   * @return A command to run the routine.
   */
  public Command sysIdRoutine() {
    return sysIdRoutine
        .quasistatic(Direction.kForward)
        .andThen(sysIdRoutine.quasistatic(Direction.kReverse))
        .andThen(sysIdRoutine.dynamic(Direction.kForward))
        .andThen(sysIdRoutine.dynamic(Direction.kReverse))
        .withName("Arm SysID");
  }

  @Override
  public void periodic() {
    armLigament.setAngle(Math.toDegrees(hardware.position()));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
