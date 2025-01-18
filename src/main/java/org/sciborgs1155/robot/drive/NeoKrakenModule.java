package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.COUPLING_RATIO;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Set;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class NeoKrakenModule implements ModuleIO {
  private final SparkMax driveMotor; // NEO
  private final SparkMaxConfig driveMotorConfig;
  private final TalonFX turnMotor; // Kraken

  private final RelativeEncoder driveEncoder;

  private final PositionVoltage rotationIn = new PositionVoltage(0);
  private final SparkClosedLoopController drivePID;
  private final SimpleMotorFeedforward driveFF;

  private final Rotation2d angularOffset;

  private double lastPosition;
  private double lastVelocity;
  private Rotation2d lastRotation;

  @Log.NT private SwerveModuleState setpoint = new SwerveModuleState();

  private final String name;

  public NeoKrakenModule(
      int drivePort, int turnPort, int sensorID, Rotation2d angularOffset, String name) {
    // Drive Motor
    driveMotor = new SparkMax(drivePort, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    drivePID = driveMotor.getClosedLoopController();
    driveFF =
        new SimpleMotorFeedforward(Driving.FF.SPARK.S, Driving.FF.SPARK.V, Driving.FF.SPARK.A);
    driveMotorConfig = new SparkMaxConfig();

    check(
        driveMotor,
        driveMotor.configure(
            driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

    driveMotorConfig.apply(
        driveMotorConfig
            .closedLoop
            .pid(Driving.PID.SPARK.P, Driving.PID.SPARK.I, Driving.PID.SPARK.D)
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder));

    driveMotorConfig.apply(
        driveMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) Driving.CURRENT_LIMIT.in(Amps)));

    driveMotorConfig.apply(
        driveMotorConfig
            .encoder
            .positionConversionFactor(Driving.POSITION_FACTOR.in(Meters))
            .velocityConversionFactor(Driving.VELOCITY_FACTOR.in(MetersPerSecond))
            .uvwAverageDepth(16)
            .uvwMeasurementPeriod(32));

    driveMotorConfig.apply(
        SparkUtils.getSignalsConfigurationFrameStrategy(
            Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
            Set.of(Sensor.INTEGRATED),
            false));

    check(
        driveMotor,
        driveMotor.configure(
            driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

    // Turn Motor

    turnMotor = new TalonFX(turnPort, "*");

    TalonFXConfiguration talonTurnConfig = new TalonFXConfiguration();

    talonTurnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonTurnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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

    register(driveMotor);
    register(turnMotor);

    TalonUtils.addMotor(turnMotor);
    resetEncoders();

    this.angularOffset = angularOffset;
    this.name = name;
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
    check(driveMotor);
    log("current", driveMotor.getOutputCurrent());
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnMotor.setVoltage(voltage);
  }

  @Override
  public double drivePosition() {
    lastPosition = SparkUtils.wrapCall(driveMotor, driveEncoder.getPosition()).orElse(lastPosition);
    // account for rotation of turn motor on rotation of drive motor
    return lastPosition - turnMotor.getPosition().getValueAsDouble() * COUPLING_RATIO;
  }

  @Override
  public double driveVelocity() {
    lastVelocity = SparkUtils.wrapCall(driveMotor, driveEncoder.getVelocity()).orElse(lastVelocity);
    return lastVelocity;
  }

  @Override
  public Rotation2d rotation() {
    lastRotation =
        Rotation2d.fromRotations(turnMotor.getPosition().getValueAsDouble()).minus(angularOffset);
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
    driveEncoder.setPosition(0);
  }

  @Override
  public void setDriveSetpoint(double velocity) {
    drivePID.setReference(
        velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, driveFF.calculate(velocity));
  }

  @Override
  public void setTurnSetpoint(double angle) {
    turnMotor.setControl(rotationIn.withPosition(angle).withSlot(0));
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
    setTurnSetpoint(setpoint.angle.getRotations());
    this.setpoint = setpoint;
  }

  @Override
  public void updateInputs(Rotation2d angle, double voltage) {
    setpoint.angle = angle;
    setDriveVoltage(voltage);
    setTurnSetpoint(angle.getRotations());
  }

  @Override
  public void close() {
    driveMotor.close();
    turnMotor.close();
  }
}
