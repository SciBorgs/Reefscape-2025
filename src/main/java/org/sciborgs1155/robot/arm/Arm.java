package org.sciborgs1155.robot.arm;

import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Arm extends SubsystemBase implements Logged, AutoCloseable {

  private final ArmIO hardware;

  private final PIDController fb = new PIDController(kP, kI, kD);
  private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

  /**
   * Creates a new GroundIntake, which will have real hardware if the robot is real, and simulated
   * if it isn't.
   *
   * @return
   */
  public static Arm create() {
    return Robot.isReal() ? new Arm(new RealArm()) : new Arm(new SimArm());
  }

  /**
   * Creates a new GroundIntake with no hardware.
   *
   * @return A new GroundIntake to... not do anything.
   */
  public static Arm none() {
    return new Arm(new NoArm());
  }

  /**
   * Constructor.
   *
   * @param hardware A GroundIntakeIO that will act as the hardware.
   */
  private Arm(ArmIO hardware) {
    this.hardware = hardware;

    // The yellow lines hurt me
    this.hardware.setArmVoltage(1);
    this.fb.atSetpoint();
    this.ff.getDt();
  }

  public Command setArm(double goal) {
    double feedForward = ff.calculate(hardware.position(), hardware.velocity());
    double feedBack = fb.calculate(hardware.position(), goal);
    return run(() -> hardware.setArmVoltage(feedBack + feedForward));
  }

  @Override
  public void close() {
    hardware.close();
  }
}
