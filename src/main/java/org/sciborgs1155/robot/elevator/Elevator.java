package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_ACCEL;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_HEIGHT;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MIN_HEIGHT;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kA;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kD;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kG;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kI;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kP;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kS;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kV;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Constants.Field.Level;
import org.sciborgs1155.robot.Robot;

public class Elevator extends SubsystemBase implements Logged, AutoCloseable {

  /**
   * Method to create a new elevator.
   *
   * @return Real or Sim elevator based on Robot.isReal().
   */
  public static Elevator create() {
    return Robot.isReal() ? new Elevator(new RealElevator()) : new Elevator(new SimElevator());
  }

  /**
   * Method to create a no elevator.
   *
   * @return No elevator object.
   */
  public static Elevator none() {
    return new Elevator(new NoElevator());
  }

  private final ElevatorIO hardware;

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
  }

  /**
   * Sets the eleveator to the max height
   *
   * @return A command to set the elevator to the max height.
   */
  public Command extend() {
    return run(() -> update(MAX_HEIGHT.in(Meters)));
  }

  /**
   * A command to fully lower the elevator.
   *
   * @return A command to fully lower the elevator.
   */
  public Command retract() {
    return run(() -> update(MIN_HEIGHT.in(Meters)));
  }

  /**
   * @param level An enum with the height of the level
   * @return A command to score at inputteed level of the reef.
   */
  public Command scoreLevel(Level level) {
    return run(() -> update(level.getHeight()));
  }

  @Log.NT
  /**
   * Method to get the positon of the elevator.
   *
   * @return Position of the elevator.
   */
  public double position() {
    return hardware.position();
  }

  @Log.NT
  /**
   * Method to get the velocity of the elevator.
   *
   * @return Velocity of the elevator.
   */
  public double velocity() {
    return hardware.velocity();
  }

  @Log.NT
  /**
   * Method to get the setpoint of the ProfiledPID.
   *
   * @return Position of the ProfiledPID.
   */
  public double positionSetpoint() {
    return pid.getSetpoint().position;
  }

  /**
   * Method to get the velocity of the setpoint of the ProfiledPID.
   *
   * @return Velocity of the ProfiledPID.
   */
  @Log.NT
  public double velocitySetpoint() {
    return pid.getSetpoint().velocity;
  }

  /**
   * Updates the position to a desired position.
   *
   * @param positionDesired position to update to.
   */
  private void update(double position) {
    position = MathUtil.clamp(position, MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters));

    double lastVelocity = pid.getSetpoint().velocity;
    double feedback = pid.calculate(hardware.position(), position);
    double feedforward = ff.calculateWithVelocities(lastVelocity, pid.getSetpoint().velocity);

    hardware.setVoltage(feedforward + feedback);
  }

  @Override
  /**
   * This method will be called periodically and is used to update the setpoint and measurement
   * lengths.
   */
  public void periodic() {
    setpoint.setLength(positionSetpoint());
    measurement.setLength(position());
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
