package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

/** Simple Arm subsystem used for climbing and intaking coral from the ground. */
public class Arm extends SubsystemBase implements Logged, AutoCloseable {
  /** Interface for interacting with the motor itself. */
  @Log.NT private final ArmIO hardware;

  private final PIDController feedbackController = new PIDController(kP, kI, kD);
  private final ArmFeedforward feedforwardController = new ArmFeedforward(kS, kG, kV, kA);

  /**
   * Returns a new {@link Arm} subsystem, which will have real hardware if the robot is real, and
   * simulated if it isn't.
   */
  public static Arm create() {
    return Robot.isReal() ? new Arm(new RealArm()) : new Arm(new SimArm());
  }

  /** Creates a new {@link Arm} with no hardware interface(does nothing). */
  public static Arm none() {
    return new Arm(new NoArm());
  }

  /**
   * Constructor.
   *
   * @param hardware
   */
  private Arm(ArmIO hardware) {
    this.hardware = hardware;
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

  /**
   * @return The position in radians/sec.
   */
  @Log.NT
  public double getVoltage() {
    return hardware.voltage();
  }

  /** Moves the arm to a specified goal angle. */
  public Command moveArmTo(Angle goal) {
    double feedForward = feedforwardController.calculate(hardware.position(), hardware.velocity());
    double feedBack = feedbackController.calculate(hardware.position(), goal.in(Radians));
    return run(() -> hardware.setVoltage(feedBack + feedForward))
        .withName("Moving Arm To: " + goal.toString() + " radians")
        .andThen(Commands.print("Yippee"));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
