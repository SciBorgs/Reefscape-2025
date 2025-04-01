package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.drive.DriveConstants.WHEEL_RADIUS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class SimModule implements ModuleIO {
  private final SwerveModuleSimulation moduleSimulation;

  private final SimulatedMotorController.GenericMotorController driveMotor;

  private final PIDController driveFeedback =
      new PIDController(Driving.PID.P, Driving.PID.I, Driving.PID.D);

  private final SimpleMotorFeedforward driveFF =
      new SimpleMotorFeedforward(
          Driving.FRONT_LEFT_FF.kS(), Driving.FRONT_LEFT_FF.kV(), Driving.FRONT_LEFT_FF.kA());

  private final SimulatedMotorController.GenericMotorController turnMotor;

  private final PIDController turnFeedback =
      new PIDController(Turning.PID.P, Turning.PID.I, Turning.PID.D);

  private SwerveModuleState setpoint = new SwerveModuleState();

  private final String name;

  public SimModule(SwerveModuleSimulation moduleSimulation, String name) {
    this.name = name;

    this.moduleSimulation = moduleSimulation;

    // configures a generic motor controller for drive motor
    // set a current limit of 60 amps
    this.driveMotor =
        moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(60));
    this.turnMotor = moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public void setDriveVoltage(double voltage) {
    this.driveMotor.requestVoltage(Volts.of(voltage));
  }

  @Override
  public void setTurnVoltage(double voltage) {
    this.turnMotor.requestVoltage(Volts.of(voltage));
  }

  @Override
  public double drivePosition() {
    return moduleSimulation.getDriveWheelFinalPosition().in(Radians) * WHEEL_RADIUS.in(Meters);
  }

  @Override
  public double driveVelocity() {
    return state().speedMetersPerSecond;
  }

  @Override
  public Rotation2d rotation() {
    return state().angle;
  }

  @Override
  public SwerveModuleState state() {
    return moduleSimulation.getCurrentState();
  }

  @Override
  public SwerveModulePosition position() {
    return new SwerveModulePosition(drivePosition(), rotation());
  }

  @Override
  public SwerveModuleState desiredState() {
    return setpoint;
  }

  @Override
  public void setDriveSetpoint(double velocity) {
    setDriveVoltage(
        driveFeedback.calculate(driveVelocity(), velocity) + driveFF.calculate(velocity));
  }

  @Override
  public void setTurnSetpoint(Rotation2d setpoint) {
    setTurnVoltage(turnFeedback.calculate(rotation().getRadians(), setpoint.getRadians()));
  }

  @Override
  public void updateSetpoint(SwerveModuleState setpoint, ControlMode mode) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint.optimize(rotation());
    // Scale setpoint by cos of turning error to reduce tread wear
    setpoint.cosineScale(rotation());

    if (mode == ControlMode.OPEN_LOOP_VELOCITY) {
      setDriveVoltage(driveFF.calculate(setpoint.speedMetersPerSecond));
    } else {
      setDriveSetpoint(setpoint.speedMetersPerSecond);
    }

    setTurnSetpoint(setpoint.angle);
    this.setpoint = setpoint;
  }

  @Override
  public void updateInputs(Rotation2d angle, double voltage) {
    setpoint.angle = angle;

    double turnVolts = turnFeedback.calculate(rotation().getRadians(), setpoint.angle.getRadians());

    setDriveVoltage(voltage);
    setTurnVoltage(turnVolts);
  }

  // Currently can not do this
  @Override
  public void resetEncoders() {}

  @Override
  public double[][] moduleOdometryData() {
    return new double[][] {};
  }

  public SwerveModulePosition[] odometryData() {
    return new SwerveModulePosition[] {};
  }

  public double[] timestamps() {
    return new double[] {};
  }

  @Override
  public void close() {}
}
