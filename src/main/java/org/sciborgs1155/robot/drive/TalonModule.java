package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class TalonModule implements ModuleIO {
  private final TalonFX driveMotor; // Kraken X60
  private final TalonFX turnMotor; // Kraken X60

  private final StatusSignal<Angle> drivePos;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Angle> turnPos;

  private final VelocityVoltage velocityOut = new VelocityVoltage(0);
  private final PositionVoltage radiansOut = new PositionVoltage(0);

  private final SimpleMotorFeedforward driveFF;

  private final Rotation2d angularOffset;

  @Log.NT private SwerveModuleState setpoint = new SwerveModuleState();

  private Rotation2d lastRotation;

  private final String name;

  public TalonModule(
      int drivePort,
      int turnPort,
      int sensorID,
      Rotation2d angularOffset,
      String name,
      boolean invert) {
    driveMotor = new TalonFX(drivePort, "*");
    drivePos = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveFF =
        new SimpleMotorFeedforward(Driving.FF.TALON.S, Driving.FF.TALON.V, Driving.FF.TALON.A);

    drivePos.setUpdateFrequency(1 / SENSOR_PERIOD.in(Seconds));
    driveVelocity.setUpdateFrequency(1 / SENSOR_PERIOD.in(Seconds));

    TalonFXConfiguration talonDriveConfig = new TalonFXConfiguration();

    talonDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonDriveConfig.Feedback.SensorToMechanismRatio = Driving.POSITION_FACTOR.in(Meters);
    talonDriveConfig.CurrentLimits.SupplyCurrentLimit = Driving.CURRENT_LIMIT.in(Amps);

    talonDriveConfig.Slot0.kP = Driving.PID.TALON.P;
    talonDriveConfig.Slot0.kI = Driving.PID.TALON.I;
    talonDriveConfig.Slot0.kD = Driving.PID.TALON.D;

    turnMotor = new TalonFX(turnPort, "*");
    turnPos = turnMotor.getPosition();

    turnPos.setUpdateFrequency(1 / SENSOR_PERIOD.in(Seconds));

    TalonFXConfiguration talonTurnConfig = new TalonFXConfiguration();

    talonTurnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonTurnConfig.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    talonTurnConfig.Feedback.RotorToSensorRatio = 1 / Turning.POSITION_FACTOR.in(Radians);
    talonTurnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    talonTurnConfig.Feedback.FeedbackRemoteSensorID = sensorID;

    talonTurnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    talonTurnConfig.Slot0.kP = Turning.PID.P;
    talonTurnConfig.Slot0.kI = Turning.PID.I;
    talonTurnConfig.Slot0.kD = Turning.PID.D;

    talonTurnConfig.CurrentLimits.StatorCurrentLimit = Turning.CURRENT_LIMIT.in(Amps);

    for (int i = 0; i < 5; i++) {
      StatusCode success = turnMotor.getConfigurator().apply(talonTurnConfig);
      if (success.isOK()) break;
    }

    driveMotor.getConfigurator().apply(talonDriveConfig);
    turnMotor.getConfigurator().apply(talonTurnConfig);

    register(turnMotor);
    register(driveMotor);

    TalonUtils.addMotor(driveMotor);
    TalonUtils.addMotor(turnMotor);

    resetEncoders();

    this.name = name;
    this.angularOffset = angularOffset;
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnMotor.setVoltage(voltage);
  }

  @Override
  public double drivePosition() {
    return drivePos.getValueAsDouble();
  }

  @Override
  public double driveVelocity() {
    return driveVelocity.getValueAsDouble();
  }

  @Override
  public Rotation2d rotation() {
    lastRotation =
        Rotation2d.fromRadians(turnMotor.getPosition().getValueAsDouble()).minus(angularOffset);
    return lastRotation;
  }

  @Override
  public SwerveModuleState state() {
    return new SwerveModuleState(driveVelocity(), rotation());
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
  public void resetEncoders() {
    driveMotor.setPosition(0);
  }

  @Override
  public void setDriveSetpoint(double velocity) {
    driveMotor.setControl(
        velocityOut.withVelocity(velocity).withFeedForward(driveFF.calculate(velocity)));
  }

  @Override
  public void setTurnSetpoint(double angle) {
    turnMotor.setControl(radiansOut.withPosition(angle));
  }

  @Override
  public void updateSetpoint(SwerveModuleState setpoint, ControlMode mode) {
    setpoint.optimize(rotation());
    // Scale setpoint by cos of turning error to reduce tread wear
    setpoint.cosineScale(rotation());

    if (mode == ControlMode.OPEN_LOOP_VELOCITY) {
      setDriveVoltage(driveFF.calculate(setpoint.speedMetersPerSecond));
    } else {
      setDriveSetpoint(setpoint.speedMetersPerSecond);
    }

    setTurnSetpoint(setpoint.angle.getRadians());
    this.setpoint = setpoint;
  }

  @Override
  public void updateInputs(Rotation2d angle, double voltage) {
    setpoint.angle = angle;
    setDriveVoltage(voltage);
    setTurnSetpoint(angle.getRadians());
  }

  @Override
  public void close() {
    turnMotor.close();
    driveMotor.close();
  }
}
