package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.arm.ArmConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.Set;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Assertion.EqualityAssertion;
import org.sciborgs1155.lib.BetterArmFeedforward;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

/** Simple Arm subsystem used for climbing and intaking coral from the ground. */
public class Arm extends SubsystemBase implements Logged, AutoCloseable {
  /** Interface for interacting with the motor itself. */
  private final ArmIO hardware;

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
  private final BetterArmFeedforward ff = new BetterArmFeedforward(kS, kG, kV, kA);

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
              DEFAULT_ANGLE.in(Degrees),
              10,
              new Color8Bit(Color.kSkyBlue)));

  private final DoubleEntry S = Tuning.entry("/Robot/tuningArm/kS", kS);
  private final DoubleEntry G = Tuning.entry("/Robot/tuningArm/kG", kG);
  private final DoubleEntry V = Tuning.entry("/Robot/tuningArm/kV", kV);
  private final DoubleEntry A = Tuning.entry("/Robot/tuningArm/kA", kA);

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
    fb.reset(position());
    fb.setGoal(DEFAULT_ANGLE.in(Radians));
    fb.enableContinuousInput(-Math.PI, Math.PI);

    setDefaultCommand(goTo(DEFAULT_ANGLE));

    this.sysIdRoutine =
        new SysIdRoutine(
            new Config(
                Volts.of(0.5).per(Second),
                Volts.of(0.2),
                Seconds.of(5),
                (state) -> SignalLogger.writeString("arm state", state.toString())),
            new Mechanism(voltage -> hardware.setVoltage(voltage.in(Volts)), null, this));

    if (TUNING) {
      SmartDashboard.putData("arm quasistatic forward", quasistaticForward());
      SmartDashboard.putData("arm quasistatic backward", quasistaticBack());
      SmartDashboard.putData("arm dynamic forward", dynamicForward());
      SmartDashboard.putData("arm dynamic backward", dynamicBack());
    }
  }

  /**
   * @return The position in radians.
   */
  @Log.NT
  public double position() {
    return hardware.position();
  }

  public void setVoltage(double volts) {
    hardware.setVoltage(volts);
  }

  /**
   * @return pose of the arm centered
   */
  @Log.NT
  public Pose3d pose() {
    return new Pose3d(AXLE_FROM_CHASSIS, new Rotation3d(0, hardware.position(), -Math.PI / 2));
  }

  public boolean atGoal() {
    return atPosition(fb.getGoal().position);
  }

  /**
   * @return The position in radians/sec.
   */
  @Log.NT
  public double velocity() {
    return hardware.velocity();
  }

  @Log.NT
  public double positionSetpoint() {
    return fb.getSetpoint().position;
  }

  @Log.NT
  public double velocitySetpoint() {
    return fb.getSetpoint().velocity;
  }

  /**
   * @param radians The position, in radians.
   * @return True if the arm's position is close enough to a given position, False if it isn't.
   */
  @Log.NT
  public boolean atPosition(double radians) {
    return Math.abs(radians - position()) < POSITION_TOLERANCE.in(Radians);
  }

  /** Moves the arm towards a specified goal angle. */
  public Command goTo(Angle goal) {
    return goTo(() -> goal.in(Radians)).withName("moving to angle");
  }

  /** Moves the arm towards a goal in radians */
  public Command goTo(DoubleSupplier goal) {
    return run(() -> {
          double lastVelocity = fb.getSetpoint().velocity;
          double position = position();
          double feedback =
              fb.calculate(
                  position,
                  MathUtil.clamp(goal.getAsDouble(), MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians)));
          double feedforward =
              ff.calculateWithVelocities(position, lastVelocity, fb.getSetpoint().velocity);
          hardware.setVoltage(feedback + feedforward);
        })
        .withName("Moving Arm To: " + goal.toString() + " radians");
  }

  public Command manualArm(InputStream input) {
    return goTo(input
            .deadband(.15, 1)
            .scale(MAX_VELOCITY.in(RotationsPerSecond))
            .scale(Constants.PERIOD.in(Seconds))
            .rateLimit(MAX_ACCEL.in(RotationsPerSecondPerSecond))
            .add(() -> fb.getGoal().position))
        .withName("manual arm");
  }

  /**
   * A Test which moves the arm towards a goal angle and then asserts that it got there.
   *
   * @param goal The goal angle to which the arm will move to.
   * @return A Test object which moves the arm and checks it got to its destination.
   */
  public Test goToTest(Angle goal) {
    Command testCommand = goTo(goal).until(fb::atGoal).withTimeout(8).withName("Arm Test");
    EqualityAssertion atGoal =
        Assertion.eAssert(
            "arm angle", () -> goal.in(Radians), this::position, POSITION_TOLERANCE.in(Radians));
    return new Test(testCommand, Set.of(atGoal));
  }

  /**
   * Moves the arm to an angle necessary to attach to the cage.
   *
   * @return A command to move the arm to be horizontal.
   */
  public Command climbSetup() {
    return goTo(CLIMB_INTAKE_ANGLE).withName("climb setup");
  }

  /**
   * Increases the current limit for the arm, then moves the arm back to climb. Keep in mind that
   * this is a one-time command, and is completely uninteruptible.
   *
   * @return A command to climb, once the climb arm is hooked onto the cage.
   */
  public Command climbExecute() {
    return runOnce(() -> hardware.setCurrentLimit(CLIMB_LIMIT))
        .andThen(goTo(CLIMB_FINAL_ANGLE))
        .finallyDo(() -> hardware.setCurrentLimit(SUPPLY_LIMIT))
        .withName("climb execute");
  }

  public Command quasistaticForward() {
    return sysIdRoutine
        .quasistatic(Direction.kForward)
        .until(() -> position() > MAX_ANGLE.in(Radians) - 0.2)
        .withName("quasistatic forward");
  }

  public Command quasistaticBack() {
    return sysIdRoutine
        .quasistatic(Direction.kReverse)
        .until(() -> position() < MIN_ANGLE.in(Radians) + 0.2)
        .withName("quasistatic backward");
  }

  public Command dynamicForward() {
    return sysIdRoutine
        .dynamic(Direction.kForward)
        .until(() -> position() > MAX_ANGLE.in(Radians) - 0.2)
        .withName("dynamic forward");
  }

  public Command dynamicBack() {
    return sysIdRoutine
        .dynamic(Direction.kReverse)
        .until(() -> position() < MIN_ANGLE.in(Radians) + 0.2)
        .withName("dynamic backward");
  }

  @Override
  public void periodic() {
    armLigament.setAngle(Math.toDegrees(position()));

    if (TUNING) {
      ff.setKa(A.get());
      ff.setKg(G.get());
      ff.setKs(S.get());
      ff.setKv(V.get());
    }
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
