package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static java.lang.Math.atan;
import static org.sciborgs1155.lib.Assertion.*;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Constants.Robot.MASS;
import static org.sciborgs1155.robot.Constants.Robot.MOI;
import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.Constants.allianceRotation;
import static org.sciborgs1155.robot.Ports.Drive.*;
import static org.sciborgs1155.robot.drive.DriveConstants.*;
import static org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving.FF_CONSTANTS;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Arrays;
import java.util.DoubleSummaryStatistics;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Assertion.EqualityAssertion;
import org.sciborgs1155.lib.Assertion.TruthAssertion;
import org.sciborgs1155.lib.FOMPoseEstimator;
import org.sciborgs1155.lib.FOMPoseEstimator.OdometryUpdate;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.lib.Tracer;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.vision.Vision.PoseEstimate;

public class Drive extends SubsystemBase implements Logged, AutoCloseable {
  // Modules
  private final ModuleIO frontLeft;
  private final ModuleIO frontRight;
  private final ModuleIO rearLeft;
  private final ModuleIO rearRight;

  @IgnoreLogged private final List<ModuleIO> modules;

  // Gyro
  private final GyroIO gyro;
  private static Rotation2d simRotation = Rotation2d.kZero;

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_OFFSET);

  private final DoubleEntry translationP =
      Tuning.entry("Robot/tuning/drive/translation p", Translation.P);
  private final DoubleEntry translationI =
      Tuning.entry("Robot/tuning/drive/translation i", Translation.I);
  private final DoubleEntry translationD =
      Tuning.entry("Robot/tuning/drive/translation d", Translation.D);

  private final DoubleEntry rotationP = Tuning.entry("Robot/tuning/drive/rotation p", Rotation.P);
  private final DoubleEntry rotationI = Tuning.entry("Robot/tuning/drive/rotation i", Rotation.I);
  private final DoubleEntry rotationD = Tuning.entry("Robot/tuning/drive/rotation d", Rotation.D);

  private final DoubleEntry maxAccel =
      Tuning.entry("Robot/tuning/drive/Max Accel", MAX_ACCEL.in(MetersPerSecondPerSecond));
  private final DoubleEntry maxSkidAccel =
      Tuning.entry(
          "Robot/tuning/drive/Max Skid Accel", MAX_SKID_ACCEL.in(MetersPerSecondPerSecond));
  private final DoubleEntry maxTiltAccel =
      Tuning.entry(
          "Robot/tuning/drive/Max Tilt Accel", MAX_TILT_ACCEL.in(MetersPerSecondPerSecond));

  private Vector<N2> desiredAcceleration = VecBuilder.fill(0, 0);

  // Odometry and pose estimation
  // private final BetterSwerveDrivePoseEstimator odometry;
  private final FOMPoseEstimator poseEstimator;

  public Vector<N3> odometryFOM() {
    return VecBuilder.fill(1, 1, Math.PI / 6)
        .times((isSkidding() ? 3 : 1) + (isColliding() ? 3 : 1));
  }

  @Log.NT private final Field2d field2d = new Field2d();
  private final FieldObject2d[] modules2d;

  // Characterization routines
  private final SysIdRoutine translationCharacterization;
  private final SysIdRoutine rotationalCharacterization;

  private Trigger colliding;

  private SwerveModulePosition[] lastPositions;
  private Rotation2d lastHeading;

  public static final ReentrantLock lock = new ReentrantLock();

  // Movement automation
  @Log.NT
  private final ProfiledPIDController translationController =
      new ProfiledPIDController(
          translationP.get(),
          translationI.get(),
          translationD.get(),
          new TrapezoidProfile.Constraints(MAX_SPEED.in(MetersPerSecond), 12));

  @Log.NT
  private final PIDController rotationController =
      new PIDController(rotationP.get(), rotationI.get(), rotationD.get());

  /**
   * A factory to create a new swerve drive based on the type of module used / real or simulation.
   */
  public static Drive create() {
    if (Robot.isReal()) {
      return new Drive(
          new ReduxGyro(),
          new TalonModule(
              FRONT_LEFT_DRIVE,
              FRONT_LEFT_TURNING,
              FRONT_LEFT_CANCODER,
              ANGULAR_OFFSETS.get(0),
              FF_CONSTANTS.get(0),
              "FL",
              false),
          new TalonModule(
              FRONT_RIGHT_DRIVE,
              FRONT_RIGHT_TURNING,
              FRONT_RIGHT_CANCODER,
              ANGULAR_OFFSETS.get(1),
              FF_CONSTANTS.get(1),
              "FR",
              true),
          new TalonModule(
              REAR_LEFT_DRIVE,
              REAR_LEFT_TURNING,
              REAR_LEFT_CANCODER,
              ANGULAR_OFFSETS.get(2),
              FF_CONSTANTS.get(2),
              "RL",
              false),
          new TalonModule(
              REAR_RIGHT_DRIVE,
              REAR_RIGHT_TURNING,
              REAR_RIGHT_CANCODER,
              ANGULAR_OFFSETS.get(3),
              FF_CONSTANTS.get(3),
              "RR",
              true));
    } else {
      return new Drive(
          new NoGyro(),
          new SimModule("FL"),
          new SimModule("FR"),
          new SimModule("RL"),
          new SimModule("RR"));
    }
  }

  /** A factory to create a nonexistent swerve drive. */
  public static Drive none() {
    return new Drive(new NoGyro(), new NoModule(), new NoModule(), new NoModule(), new NoModule());
  }

  /** A swerve drive subsystem containing four {@link ModuleIO} modules and a gyroscope. */
  public Drive(
      GyroIO gyro, ModuleIO frontLeft, ModuleIO frontRight, ModuleIO rearLeft, ModuleIO rearRight) {
    this.gyro = gyro;
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.rearLeft = rearLeft;
    this.rearRight = rearRight;

    modules = List.of(this.frontLeft, this.frontRight, this.rearLeft, this.rearRight);
    modules2d = new FieldObject2d[modules.size()];
    lastPositions = modulePositions();
    lastHeading = gyro.rotation2d();

    translationCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("translation state", state.toString())),
            new SysIdRoutine.Mechanism(
                volts ->
                    modules.forEach(
                        m -> m.updateInputs(Rotation2d.fromRadians(0), volts.in(Volts))),
                null,
                this,
                "translation"));
    rotationalCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("rotation state", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> {
                  this.frontLeft.updateInputs(
                      Rotation2d.fromRadians(3 * Math.PI / 4), volts.in(Volts));
                  this.frontRight.updateInputs(
                      Rotation2d.fromRadians(Math.PI / 4), volts.in(Volts));
                  this.rearLeft.updateInputs(
                      Rotation2d.fromRadians(-3 * Math.PI / 4), volts.in(Volts));
                  this.rearRight.updateInputs(
                      Rotation2d.fromRadians(-Math.PI / 4), volts.in(Volts));
                },
                null,
                this,
                "rotation"));

    poseEstimator =
        new FOMPoseEstimator(
            new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)),
            kinematics,
            modulePositions());

    for (int i = 0; i < modules.size(); i++) {
      var module = modules.get(i);
      modules2d[i] = field2d.getObject("module-" + module.name());
    }

    translationController.setTolerance(Translation.TOLERANCE.in(Meters));
    rotationController.enableContinuousInput(0, 2 * Math.PI);
    rotationController.setTolerance(Rotation.TOLERANCE.in(Radians));
    // configure acceleration limiting and change according to conditions

    colliding = new Trigger(this::isColliding);

    colliding.onTrue(
        runOnce(() -> maxAccel.set(MAX_ACCEL.times(3).in(MetersPerSecondPerSecond)))
            .andThen(
                Commands.waitSeconds(4)
                    .andThen(() -> maxAccel.set(MAX_ACCEL.in(MetersPerSecondPerSecond))))
            .asProxy());

    Function<ModuleIO, BooleanSupplier> stalling = 
      m -> 
         new Trigger(() -> m.desiredState().speedMetersPerSecond > 0.3 && m.state().speedMetersPerSecond < 0.1)
         .debounce(0.2, DebounceType.kRising).debounce(0.08, DebounceType.kFalling);
         

    FaultLogger.register(
        this::isColliding,
        "Colliding",
        "The robot is accelerating significantly higher than the max acceleration",
        FaultType.INFO);
    FaultLogger.register(
        this::isSkidding,
        "Skidding",
        "One wheel is moving significantly faster than the others.",
        FaultType.INFO);
    TalonOdometryThread.getInstance().start();

    if (TUNING) {
      SmartDashboard.putData(
          "translation quasistatic forward",
          translationCharacterization
              .quasistatic(Direction.kForward)
              .withName("translation quasistatic forward"));
      SmartDashboard.putData(
          "translation dynamic forward",
          translationCharacterization
              .dynamic(Direction.kForward)
              .withName("translation dynamic forward"));
      SmartDashboard.putData(
          "translation quasistatic backward",
          translationCharacterization.quasistatic(Direction.kReverse));
      SmartDashboard.putData(
          "translation dynamic backward",
          translationCharacterization
              .dynamic(Direction.kReverse)
              .withName("translation quasistatic backward"));
      SmartDashboard.putData(
          "rotation quasistatic forward",
          rotationalCharacterization.quasistatic(Direction.kForward));
      SmartDashboard.putData(
          "rotation dynamic forward",
          rotationalCharacterization
              .dynamic(Direction.kForward)
              .withName("rotation quasistatic forward"));
      SmartDashboard.putData(
          "rotation quasistatic backward",
          rotationalCharacterization
              .quasistatic(Direction.kReverse)
              .withName("rotation quasistatic backward"));
      SmartDashboard.putData(
          "rotation dynamic backward",
          rotationalCharacterization
              .dynamic(Direction.kReverse)
              .withName("rotation dynamic backward"));
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @Log.NT
  public Pose2d pose() {
    return poseEstimator.pose();
  }

  /** Returns a Pose3D of the estimated pose of the robot. */
  public Pose3d pose3d() {
    return new Pose3d(pose());
  }

  /**
   * Returns the currently-estimated field-relative yaw of the robot.
   *
   * @return The rotation.
   */
  @Log.NT
  public Rotation2d heading() {
    return pose().getRotation();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  /**
   * Drives the robot based on a {@link InputStream} for field relative x y and omega velocities.
   *
   * @param vx A supplier for the velocity of the robot along the x axis (perpendicular to the
   *     alliance side).
   * @param vy A supplier for the velocity of the robot along the y axis (parallel to the alliance
   *     side).
   * @param vOmega A supplier for the angular velocity of the robot.
   * @param elevatorHeight A supplier for the current height of the elevator.
   * @return The driving command.
   */
  /** /** Drive command with acceleration limiting. */
  public Command drive(
      DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vOmega, DoubleSupplier elevatorHeight) {
    return run(
        () -> {
          // Set chassis speeds
          setChassisSpeeds(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  vx.getAsDouble(),
                  vy.getAsDouble(),
                  vOmega.getAsDouble(),
                  heading().plus(allianceRotation())),
              ControlMode.OPEN_LOOP_VELOCITY,
              elevatorHeight.getAsDouble());
        });
  }

  /**
   * Drives the robot based on a {@link InputStream} for field relative x y and omega velocities.
   *
   * @param vx A supplier for the velocity of the robot along the x axis (perpendicular to the
   *     alliance side).
   * @param vy A supplier for the velocity of the robot along the y axis (parallel to the alliance
   *     side).
   * @param heading A supplier for the field relative heading of the robot.
   * @param elevatorHeight A supplier for the current height of the elevator.
   * @return The driving command.
   */
  public Command drive(
      DoubleSupplier vx,
      DoubleSupplier vy,
      Supplier<Rotation2d> heading,
      DoubleSupplier elevatorHeight) {
    return drive(
            vx,
            vy,
            () -> rotationController.calculate(heading().getRadians(), heading.get().getRadians()),
            elevatorHeight)
        .beforeStarting(rotationController::reset);
  }

  /**
   * Drives the robot based in a {@link InputStream} for field-relative x, y, and omega velocities.
   * Also adds a little extra translational velocity to move the robot to a certain desired position
   * if the driver is already attempting to move in that general direction. This command does not
   * assist in controlling the rotation of the robot.
   *
   * @param vx A supplier for the velocity of the robot along the x axis (perpendicular to the
   *     alliance side).
   * @param vy A supplier for the velocity of the robot along the y axis (parallel to the alliance
   *     side).
   * @param vOmega A supplier for the angular velocity of the robot.
   * @param target The target field-relative position for the robot, as a {@link Translation2d}.
   * @param elevatorHeight A supplier for the current height of the elevator.
   * @return The assisted driving command.
   */
  public Command assistedDrive(
      DoubleSupplier vx,
      DoubleSupplier vy,
      DoubleSupplier vOmega,
      Translation2d target,
      DoubleSupplier elevatorHeight) {
    return run(() -> {
          Vector<N2> driverVel = VecBuilder.fill(vx.getAsDouble(), vy.getAsDouble());
          Vector<N2> displacement = pose().getTranslation().minus(target).toVector();
          Vector<N2> perpDisplacement = displacement.projection(driverVel).minus(displacement);
          Vector<N2> result =
              driverVel.plus(
                  perpDisplacement
                      .unit()
                      .times(translationController.calculate(perpDisplacement.norm(), 0)));
          setChassisSpeeds(
              Math.acos(driverVel.unit().dot(displacement.unit()))
                          < ASSISTED_DRIVING_THRESHOLD.in(Radians)
                      && !Double.isNaN(result.norm())
                  ? ChassisSpeeds.fromFieldRelativeSpeeds(
                      result.get(0),
                      result.get(1),
                      vOmega.getAsDouble(),
                      heading().plus(allianceRotation()))
                  : ChassisSpeeds.fromFieldRelativeSpeeds(
                      vx.getAsDouble(),
                      vy.getAsDouble(),
                      vOmega.getAsDouble(),
                      heading().plus(allianceRotation())),
              ControlMode.CLOSED_LOOP_VELOCITY,
              elevatorHeight.getAsDouble());
        })
        .repeatedly();
  }

  /**
   * Drives the robot based in a {@link InputStream} for field relative x y and omega velocities.
   * Also adds a little translational velocity to move the robot to a certain desired position if
   * the driver is already attempting to move in that general direction. If the driver is not
   * currently attempting to rotate the robot, this command will also automatically rotate the robot
   * to a desired heading.
   *
   * @param vx A supplier for the velocity of the robot along the x axis (perpendicular to the
   *     alliance side).
   * @param vy A supplier for the velocity of the robot along the y axis (parallel to the alliance
   *     side).
   * @param vOmega A supplier for the angular velocity of the robot.
   * @param target The target field-relative position for the robot, as a {@link Pose2d}.
   * @param elevatorHeight A supplier for the current height of the elevator.
   * @return The assisted driving command.
   */
  public Command assistedDrive(
      DoubleSupplier vx,
      DoubleSupplier vy,
      DoubleSupplier vOmega,
      Pose2d target,
      DoubleSupplier elevatorHeight) {
    return assistedDrive(
            vx,
            vy,
            () ->
                Math.abs(target.getRotation().getRadians() - heading().getRadians())
                        > Rotation.TOLERANCE.in(Radians)
                    ? rotationController.calculate(
                        heading().minus(target.getRotation()).getRadians(), 0)
                    : vOmega.getAsDouble(),
            target.getTranslation(),
            elevatorHeight)
        .until(() -> vOmega.getAsDouble() > ASSISTED_ROTATING_THRESHOLD)
        .andThen(assistedDrive(vx, vy, vOmega, target.getTranslation(), elevatorHeight));
  }

  /**
   * Drives the robot while facing a target pose.
   *
   * @param vx A supplier for the absolute x velocity of the robot.
   * @param vy A supplier for the absolute y velocity of the robot.
   * @param translation A supplier for the translation2d to face on the field.
   * @param elevatorHeight A supplier for the current height of the elevator.
   * @return A command to drive while facing a target.
   */
  public Command driveFacingTarget(
      DoubleSupplier vx,
      DoubleSupplier vy,
      Supplier<Translation2d> translation,
      DoubleSupplier elevatorHeight) {
    return drive(
        vx, vy, () -> translation.get().minus(pose().getTranslation()).getAngle(), elevatorHeight);
  }

  @Log.NT
  public boolean atRotationalSetpoint() {
    return rotationController.atSetpoint();
  }

  /**
   * Checks whether the robot is facing towards a point on the field.
   *
   * @param target The field-relative point to check.
   * @return Whether the robot is facing the target closely enough.
   */
  public boolean isFacing(Translation2d target) {
    return Math.abs(
            lastHeading.getRadians()
                - target.minus(pose().getTranslation()).getAngle().getRadians())
        < rotationController.getErrorTolerance();
  }

  /**
   * Checks whether the robot is at a certain field coordinate.
   *
   * @param position The target position.
   * @param tolerance The tolerance, above which the robot will not be considered to be "at" the
   *     target.
   * @return Whether the robot is at a target translation.
   */
  public boolean atTranslation(Translation2d position, Distance tolerance) {
    return pose().getTranslation().minus(position).getNorm() < tolerance.in(Meters);
  }

  /**
   * Checks whether the robot is at a certain rotation.
   *
   * @param rotation The target rotation.
   * @param tolerance The tolerance, above which the robot will not be considered to be "at" the
   *     rotation.
   * @return Whether the robot is at a target rotation.
   */
  public boolean atRotation(Rotation2d rotation, Angle tolerance) {
    return Math.abs(heading().minus(rotation).getRadians()) < tolerance.in(Radians);
  }

  /**
   * Checks whether the robot is at a certain pose.
   *
   * @param pose The target pose.
   * @param translationTolerance The translational tolerance.
   * @param rotationTolerance The rotational tolerance.
   * @return Whether the robot is at a target pose.
   */
  public boolean atPose(Pose2d pose, Distance translationTolerance, Angle rotationTolerance) {
    return atTranslation(pose.getTranslation(), translationTolerance)
        && atRotation(pose.getRotation(), rotationTolerance);
  }

  /**
   * Checks whether the robot is at a certain pose. Uses the DriveConstants tolerances.
   *
   * @param pose The target pose.
   * @return Whether the robot is at a target pose.
   */
  public boolean atPose(Pose2d pose) {
    return atPose(pose, Translation.TOLERANCE, Rotation.TOLERANCE);
  }

  /**
   * Sets the states of each swerve module using target speeds that the drivetrain will work to
   * reach. Applies acceleration limits to ensure smooth and safe operation.
   *
   * @param desired The robot relative speeds the drivetrain will run at.
   * @param mode The control loop used to achieve those speeds.
   * @param elevatorHeight A supplier for the current height of the elevator.
   */
  public void setChassisSpeeds(ChassisSpeeds desired, ControlMode mode, double elevatorHeight) {
    Vector<N2> currentVelocity =
        VecBuilder.fill(
            robotRelativeChassisSpeeds().vxMetersPerSecond,
            robotRelativeChassisSpeeds().vyMetersPerSecond);

    Vector<N2> deltaV =
        VecBuilder.fill(desired.vxMetersPerSecond, desired.vyMetersPerSecond)
            .minus(currentVelocity);
    // Vector<N2> limitedVelocity =
    // currentVelocity.plus(
    //     currentVelocity.norm() > 1e-6
    //         ? forwardAccelerationLimit(
    //             skidAccelerationLimit(
    //                 tiltAccelerationLimit(
    //                     skidAccelerationLimit(forwardAccelerationLimit(accel)),
    //                     elevatorHeight.getAsDouble())))
    //         : accel);
    Vector<N2> limitedDeltaV = skidAccelerationLimit(deltaV);

    Vector<N2> limitedVelocity = currentVelocity.plus(limitedDeltaV);
    desiredAcceleration = limitedDeltaV.div(PERIOD.in(Seconds));

    log("forward accel limit", (skidAccelerationLimit(deltaV)).norm());

    ChassisSpeeds newSpeeds =
        new ChassisSpeeds(
            limitedVelocity.get(0), limitedVelocity.get(1), desired.omegaRadiansPerSecond);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(newSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED.in(MetersPerSecond));
    setModuleStates(
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                kinematics.toChassisSpeeds(states), Constants.PERIOD.in(Seconds))),
        mode);
  }

  /**
   * Applies forward acceleration limiting to the desired acceleration based on the current
   * velocity. Limits the acceleration in the direction of the current velocity to prevent excessive
   * acceleration.
   *
   * @param desiredAccel The desired field-relative acceleration vector.
   * @return The adjusted acceleration vector after applying forward acceleration limits.
   */
  private Vector<N2> forwardAccelerationLimit(Vector<N2> deltaV) {
    Vector<N2> currVel =
        VecBuilder.fill(
            fieldRelativeChassisSpeeds().vxMetersPerSecond,
            fieldRelativeChassisSpeeds().vyMetersPerSecond);
    double limit =
        maxAccel.get()
            * PERIOD.in(Seconds)
            * (1 - Math.min(1, (currVel.norm() / MAX_SPEED.in(MetersPerSecond))));
    log("accel limit", limit);
    Vector<N2> proj = deltaV.projection(currVel);
    if (proj.norm() > limit && proj.dot(currVel) > 0) {
      Vector<N2> parallel = proj.unit().times(limit);
      Vector<N2> perpendicular = deltaV.minus(proj);
      return parallel.plus(perpendicular);
    }
    return deltaV;
  }

  // /**
  //  * Applies tilt acceleration limiting based on the height of the elevator. The higher the
  //  * elevator, the more the acceleration is limited to prevent tipping.
  //  *
  //  * @param desiredAccel The desired field-relative acceleration vector.
  //  * @param elevatorHeight The current height of the elevator.
  //  * @return The adjusted acceleration vector after applying tilt acceleration limits.
  //  */
  // private Vector<N2> tiltAccelerationLimit(Vector<N2> desiredAccel, double elevatorHeight) {
  //   double limit =
  //       MAX_TILT_ACCEL.in(MetersPerSecondPerSecond)
  //           * (1 - (elevatorHeight / MAX_EXTENSION.in(Meters)));
  //   return desiredAccel.norm() > limit && elevatorHeight > MIN_EXTENSION.in(Meters)
  //       ? desiredAccel.unit().times(limit)
  //       : desiredAccel;
  // }

  /**
   * Applies skid acceleration limiting based on the maximum allowed skid acceleration. Ensures that
   * the acceleration does not exceed the skid limit to prevent skidding.
   *
   * @param desiredAccel The desired field-relative acceleration vector.
   * @return The adjusted acceleration vector after applying skid acceleration limits.
   */
  private Vector<N2> skidAccelerationLimit(Vector<N2> deltaV) {
    return deltaV.norm() == 0
        ? deltaV
        : deltaV.unit().times(Math.min(deltaV.norm(), maxSkidAccel.get() * PERIOD.in(Seconds)));
  }

  /**
   * Sets the states of each of the swerve modules.
   *
   * @param desiredStates The desired SwerveModule states.
   * @param mode The method to use when controlling the drive motor.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, ControlMode mode) {
    if (desiredStates.length != modules.size()) {
      throw new IllegalArgumentException("desiredStates must have the same length as modules");
    }

    for (int i = 0; i < modules.size(); i++) {
      modules.get(i).updateSetpoint(desiredStates[i], mode);
    }
  }

  /**
   * Command factory that automatically path-follows, in a straight line, to a position on the
   * field.
   *
   * @param target The pose to reach.
   * @return The command to run the control loop until the pose is reached.
   */
  public Command driveTo(Supplier<Pose2d> target) {
    return run(() -> {
          Pose2d targetPose = target.get();
          Pose2d pose = pose();
          Vector<N3> difference =
              VecBuilder.fill(
                  pose.getX() - targetPose.getX(),
                  pose.getY() - targetPose.getY(),
                  MathUtil.angleModulus(
                          pose.getRotation().getRadians() - targetPose.getRotation().getRadians())
                      * RADIUS.in(Meters));
          double out = translationController.calculate(difference.norm(), 0);
          Vector<N3> velocities = difference.unit().times(out);
          log("driveTo goal", targetPose);
          setChassisSpeeds(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  velocities.get(0),
                  velocities.get(1),
                  velocities.get(2) / RADIUS.in(Meters),
                  pose().getRotation()),
              ControlMode.CLOSED_LOOP_VELOCITY,
              0);
        })
        .until(() -> atPose(target.get(), Translation.TOLERANCE, Rotation.TOLERANCE))
        .andThen(stop())
        .withName("drive to pose");
  }

  public Command driveTo(Pose2d goal) {
    return driveTo(() -> goal);
  }

  /**
   * Follows a given pathplanner path.
   *
   * @param path A pathplanner path.
   * @param elevatorHeight A supplier for the current height of the elevator.
   * @return A command to follow a path.
   */
  public Command pathfollow(PathPlannerPath path, DoubleSupplier elevatorHeight) {
    return new FollowPathCommand(
        path,
        this::pose,
        this::robotRelativeChassisSpeeds,
        (ChassisSpeeds a, DriveFeedforwards b) ->
            setChassisSpeeds(a, ControlMode.CLOSED_LOOP_VELOCITY, elevatorHeight.getAsDouble()),
        new PPHolonomicDriveController(
            new PIDConstants(Translation.P, Translation.I, Translation.D),
            new PIDConstants(Rotation.P, Rotation.I, Rotation.D)),
        new RobotConfig(
            MASS,
            MOI,
            new ModuleConfig(
                WHEEL_RADIUS,
                MAX_SPEED,
                WHEEL_COF,
                DCMotor.getKrakenX60(1),
                DriveConstants.ModuleConstants.Driving.GEARING,
                DriveConstants.ModuleConstants.Driving.STATOR_LIMIT,
                1),
            DriveConstants.TRACK_WIDTH),
        () -> false,
        this);
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules.get(i).drivePosition();
    }
    return values;
  }

  /**
   * @return If the robot is skidding.
   */
  @Log.NT
  public boolean isSkidding() {
    DoubleSummaryStatistics diffs =
        Arrays.stream(moduleStates())
            .mapToDouble(
                s ->
                    Constants.fromPolarCoords(s.speedMetersPerSecond, s.angle)
                        .minus(
                            VecBuilder.fill(
                                robotRelativeChassisSpeeds().vxMetersPerSecond,
                                robotRelativeChassisSpeeds().vyMetersPerSecond))
                        .norm())
            .summaryStatistics();
    return diffs.getMax() - diffs.getMin() > SKIDDING_THRESHOLD.in(MetersPerSecond);
  }

  /**
   * @return If the robot is colliding.
   */
  @Log.NT
  public boolean isColliding() {
    log("accel", gyro.acceleration().norm());
    return gyro.acceleration().norm() - desiredAcceleration.norm()
        > COLLISION_THRESHOLD.in(MetersPerSecondPerSecond);
  }

  /** Resets all drive encoders to read a position of 0. */
  public void resetEncoders() {
    modules.forEach(ModuleIO::resetEncoders);
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeading() {
    return runOnce(gyro::reset);
  }

  /** Returns the module states. */
  @Log.NT
  public SwerveModuleState[] moduleStates() {
    return modules.stream().map(ModuleIO::state).toArray(SwerveModuleState[]::new);
  }

  /** Returns the module states. */
  @Log.NT
  private SwerveModuleState[] moduleSetpoints() {
    return modules.stream().map(ModuleIO::desiredState).toArray(SwerveModuleState[]::new);
  }

  /** Returns the module positions. */
  @Log.NT
  public SwerveModulePosition[] modulePositions() {
    return modules.stream().map(ModuleIO::position).toArray(SwerveModulePosition[]::new);
  }

  /** Returns the robot-relative chassis speeds. */
  public ChassisSpeeds robotRelativeChassisSpeeds() {
    return kinematics.toChassisSpeeds(moduleStates());
  }

  /** Returns the field-relative chassis speeds. */
  public ChassisSpeeds fieldRelativeChassisSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeChassisSpeeds(), heading());
  }

  @Log.NT
  private double odomFOM() {
    return 1
        - (isSkidding() ? 0.6 : 0) // reduce FOM if skidding
        - (isColliding() ? 0.3 : 0); // reduce FOM if colliding
  }

  /**
   * Drives the robot to a Choreo {@link SwerveSample}.
   *
   * @param sample The SwerveSample to drive the robot to.
   * @param rotation A goal rotation to drive to.
   * @param elevatorHeight A supplier for the current height of the elevator.
   */
  public void goToSample(SwerveSample sample, Rotation2d rotation, DoubleSupplier elevatorHeight) {
    Vector<N2> displacement =
        pose().getTranslation().minus(sample.getPose().getTranslation()).toVector();

    Vector<N2> result =
        VecBuilder.fill(sample.vx, sample.vy)
            .plus(
                displacement.norm() > 1e-4
                    ? displacement
                        .unit()
                        .times(translationController.calculate(displacement.norm(), 0))
                    : displacement.times(0));

    if (Double.isNaN(result.norm())) {
      FaultLogger.report(
          "Alignment interference",
          "Assisted Drive and Pathfinding are interfering with each other, causing a NaN result speed.\nSpeed defaulted to zero.",
          FaultType.WARNING);
      result = VecBuilder.fill(0, 0);
    }

    setChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            result.get(0),
            result.get(1),
            rotationController.calculate(heading().minus(rotation).getRadians(), 0),
            heading()),
        DRIVE_MODE,
        elevatorHeight.getAsDouble());
  }

  /**
   * Adds on a Choreo {@link SwerveSample} to the drive's desired velocity such that it does not
   * interfere (as much) with driving.
   *
   * @param vx Driver's inputted vx.
   * @param vy Driver's inputted vy.
   * @param omega Driver's inputted omega.
   * @param sample The swerve sample being added on.
   * @param elevatorHeight A supplier for the height of the elevator.
   */
  public void addOnSample(
      DoubleSupplier vx,
      DoubleSupplier vy,
      DoubleSupplier omega,
      SwerveSample sample,
      DoubleSupplier elevatorHeight) {
    Vector<N2> driverSpeeds = VecBuilder.fill(vx.getAsDouble(), vy.getAsDouble());
    Vector<N2> sampleSpeeds = VecBuilder.fill(sample.vx, sample.vy);
    Vector<N2> speeds =
        driverSpeeds.norm() > 1e-3 && driverSpeeds.dot(sampleSpeeds) > 0
            ? sampleSpeeds.plus(driverSpeeds).projection(driverSpeeds)
            : VecBuilder.fill(0, 0);
    setChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.get(0), speeds.get(1), omega.getAsDouble(), heading()),
        DRIVE_MODE,
        elevatorHeight.getAsDouble());
  }

  /**
   * Updates pose estimate based on wheel odometry and vision-provided {@link EstimatedRobotPose}s.
   *
   * @param poses The pose estimates based on vision data.
   */
  public void updatePoseEstimate(PoseEstimate... poses) {
    Tracer.startTrace("drive pd");
    if (Robot.isReal()) {
      lock.lock();
      try {
        double[] timestamps = modules.get(2).timestamps();

        // get the positions of all modules at a given timestamp [[module0 odometry], [module1
        // odometry], ...]
        SwerveModulePosition[][] allPositions =
            new SwerveModulePosition[][] {
              modules.get(0).odometryData(),
              modules.get(1).odometryData(),
              modules.get(2).odometryData(),
              modules.get(3).odometryData(),
            };
        double[][] allGyro = gyro.odometryData();

        OdometryUpdate[] odometryUpdates = new OdometryUpdate[timestamps.length];

        for (int i = 0; i < timestamps.length; i++) {
          SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
          for (int m = 0; m < modules.size(); m++) {
            modulePositions[m] = allPositions[m][i];
          }

          Rotation2d angle = Rotation2d.fromRotations(allGyro[0][i]);

          odometryUpdates[i] = new OdometryUpdate(timestamps[i], angle, modulePositions);
          lastPositions = modulePositions;
          lastHeading = angle;
        }

        poseEstimator.update(odometryUpdates, odometryFOM(), poses);

      } catch (Exception e) {
        e.printStackTrace();
      } finally {
        lock.unlock();
      }
    } else {
      poseEstimator.updateNoVision(simRotation, modulePositions());
      lastPositions = modulePositions();
    }

    Pose3d[] loggedEstimates = new Pose3d[poses.length];

    for (int i = 0; i < poses.length; i++) {
      loggedEstimates[i] = poses[i].estimatedPose().estimatedPose;
      field2d
          .getObject("Cam " + i + " Est Pose")
          .setPose(poses[i].estimatedPose().estimatedPose.toPose2d());
    }

    log("estimated poses", loggedEstimates);

    Tracer.endTrace();
  }

  @Override
  public void periodic() {
    // update our heading in reality / sim

    // update our simulated field poses
    field2d.setRobotPose(pose());

    for (int i = 0; i < modules2d.length; i++) {
      var module = modules.get(i);
      var transform = new Transform2d(MODULE_OFFSET[i], module.position().angle);
      modules2d[i].setPose(pose().transformBy(transform));
    }

    if (TUNING) {
      translationController.setPID(translationP.get(), translationI.get(), translationD.get());
      rotationController.setPID(rotationP.get(), rotationI.get(), rotationD.get());
    }

    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }

  @Override
  public void simulationPeriodic() {
    simRotation =
        simRotation.rotateBy(
            Rotation2d.fromRadians(
                !Double.isNaN(robotRelativeChassisSpeeds().omegaRadiansPerSecond)
                    ? robotRelativeChassisSpeeds().omegaRadiansPerSecond
                        * Constants.PERIOD.in(Seconds)
                    : 0));
  }

  /** Stops the drivetrain. */
  public Command stop() {
    return runOnce(() -> setChassisSpeeds(new ChassisSpeeds(), ControlMode.OPEN_LOOP_VELOCITY, 0));
  }

  /** Sets the drivetrain to an "X" configuration, preventing movement. */
  public Command lock() {
    var front = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    var back = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    return run(
        () ->
            setModuleStates(
                new SwerveModuleState[] {front, back, back, front},
                ControlMode.OPEN_LOOP_VELOCITY));
  }

  /**
   * Factory for our drive systems check.
   *
   * <p>Checks for properly functioning movement and speed / heading measurements.
   *
   * @return The test to run.
   */
  public Test systemsCheck() {
    ChassisSpeeds speeds = new ChassisSpeeds(1, 1, 0);
    Command testCommand =
        run(() -> setChassisSpeeds(speeds, ControlMode.OPEN_LOOP_VELOCITY, 0)).withTimeout(0.75);
    Function<ModuleIO, TruthAssertion> speedCheck =
        m ->
            tAssert(
                () -> m.state().speedMetersPerSecond * Math.signum(m.position().angle.getCos()) > 1,
                "Drive Syst Check " + m.name() + " Module Speed",
                () -> "expected: >= 1; actual: " + m.state().speedMetersPerSecond);
    Function<ModuleIO, EqualityAssertion> atAngle =
        m ->
            eAssert(
                "Drive Syst Check " + m.name() + " Module Angle (degrees)",
                () -> 45,
                () -> Units.radiansToDegrees(atan(m.position().angle.getTan())),
                1);
    Set<Assertion> assertions =
        modules.stream()
            .flatMap(m -> Stream.of(speedCheck.apply(m), atAngle.apply(m)))
            .collect(Collectors.toSet());
    return new Test(testCommand, assertions);
  }

  public void close() throws Exception {
    frontLeft.close();
    frontRight.close();
    rearLeft.close();
    rearRight.close();
    gyro.close();
  }
}
