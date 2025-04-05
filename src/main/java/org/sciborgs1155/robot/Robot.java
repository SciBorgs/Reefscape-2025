package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.disabled;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.teleop;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.test;
import static org.sciborgs1155.robot.Constants.DEADBAND;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Constants.ROBOT_TYPE;
import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.Constants.alliance;
import static org.sciborgs1155.robot.arm.ArmConstants.CORAL_INTAKE_ANGLE;
import static org.sciborgs1155.robot.arm.ArmConstants.DEFAULT_ANGLE;
import static org.sciborgs1155.robot.arm.ArmConstants.TROUGH_OUTTAKE_ANGLE;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_ANGULAR_ACCEL;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_SPEED;
import static org.sciborgs1155.robot.drive.DriveConstants.TELEOP_ANGULAR_SPEED;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Arrays;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.FieldConstants.Face.Side;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.arm.Arm;
import org.sciborgs1155.robot.commands.Alignment;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.commands.Corolling;
import org.sciborgs1155.robot.commands.Dashboard;
import org.sciborgs1155.robot.commands.Scoraling;
import org.sciborgs1155.robot.coroller.Coroller;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.ElevatorConstants;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.led.LEDs;
import org.sciborgs1155.robot.scoral.Scoral;
import org.sciborgs1155.robot.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class Robot extends CommandRobot {
  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);
  private final boolean dashboardConfig = Dashboard.configure();

  private final PowerDistribution pdh = new PowerDistribution();

  // SUBSYSTEMS
  private final Drive drive =
      switch (ROBOT_TYPE) {
        case FULL, SCORALING, COROLLING, CHASSIS -> Drive.create();
        default -> Drive.none();
      };

  private final Vision vision =
      switch (ROBOT_TYPE) {
        case FULL, SCORALING, COROLLING, CHASSIS -> Vision.create();
        default -> Vision.none();
      };

  @Logged
  private final Elevator elevator =
      switch (ROBOT_TYPE) {
        case FULL, SCORALING -> Elevator.create();
        default -> Elevator.none();
      };

  @Logged
  private final Scoral scoral =
      switch (ROBOT_TYPE) {
        case FULL, SCORALING -> Scoral.create();
        default -> Scoral.none();
      };

  @Logged
  private final Hopper hopper =
      switch (ROBOT_TYPE) {
        case FULL, SCORALING -> Hopper.create();
        default -> Hopper.none();
      };

  @Logged
  private final Coroller coroller =
      switch (ROBOT_TYPE) {
        case FULL, COROLLING -> Coroller.create();
        default -> Coroller.none();
      };

  @Logged
  private final Arm arm =
      switch (ROBOT_TYPE) {
        case FULL, COROLLING -> Arm.create();
        default -> Arm.none();
      };

  private final LEDs leds = LEDs.create();

  private final Scoraling scoraling = new Scoraling(hopper, scoral, elevator, leds);
  private final Corolling corolling = new Corolling(arm, coroller);

  // COMMANDS
  private final Alignment align = new Alignment(drive, elevator, scoral, leds);

  @NotLogged
  private final SendableChooser<Command> autos =
      Autos.configureAutos(drive, scoraling, elevator, align, scoral);

  @Logged private double speedMultiplier = Constants.FULL_SPEED_MULTIPLIER;

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    super(PERIOD.in(Seconds));
    configureGameBehavior();
    configureBindings();

    // Warmup pathfinding commands, as the first run could have significant delays.
    Commands.waitSeconds(3).andThen(align.warmupCommand()).schedule();
    // Wait to set thread priority so that vendor threads can initialize
    // Commands.sequence(
    //         Commands.waitSeconds(10),
    //         // Danger: may result in other threads (logging, vendor status frames) being delayed
    //         Commands.runOnce(() -> Threads.setCurrentThreadPriority(true, 10)))
    //     .ignoringDisable(true)
    //     .schedule();
  }

  /** Configures basic behavior for different periods during the game. */
  private void configureGameBehavior() {
    // Configure logging with DataLogManager, Monologue, and FaultLogger
    DataLogManager.start("/u/logs");
    // SignalLogger.enableAutoLogging(true);
    // SignalLogger.setPath("/u/logs");
    addPeriodic(FaultLogger::update, 2);

    Epilogue.bind(this);
    Epilogue.configure(
        config -> {
          config.backend =
              DriverStation.isFMSAttached()
                  ? new FileBackend(DataLogManager.getLog())
                  : new NTEpilogueBackend(NetworkTableInstance.getDefault());
        });

    FaultLogger.register(pdh);
    SmartDashboard.putData("Auto Chooser", autos);

    if (TUNING) {
      addPeriodic(
          () ->
              Epilogue.getConfig()
                  .backend
                  .log(
                      "/Robot/camera transforms",
                      Arrays.stream(vision.cameraTransforms())
                          .map(
                              t ->
                                  new Pose3d(
                                      drive
                                          .pose3d()
                                          .getTranslation()
                                          .plus(
                                              t.getTranslation()
                                                  .rotateBy(drive.pose3d().getRotation())),
                                      t.getRotation().plus(drive.pose3d().getRotation())))
                          .toArray(Pose3d[]::new),
                      Pose3d.struct),
          PERIOD.in(Seconds));
    }

    // Configure pose estimation updates from vision every tick
    // addPeriodic(() -> vision.feedEstimatorHeading(drive.heading()), PERIOD);

    addPeriodic( // unused
        () -> drive.updateEstimates(vision.estimatedGlobalPoses(Rotation2d.kZero)), PERIOD);

    RobotController.setBrownoutVoltage(6.0);

    if (isReal()) {
      pdh.clearStickyFaults();
      pdh.setSwitchableChannel(true);
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
      addPeriodic(() -> vision.simulationPeriodic(drive.pose()), PERIOD.in(Seconds));
    }

    addPeriodic(() -> Dashboard.tick(), PERIOD.in(Seconds));
    addPeriodic(() -> Dashboard.setElevatorEntry(elevator.position()), PERIOD.in(Seconds));
  }

  /** Configures trigger -> command bindings. */
  private void configureBindings() {
    InputStream raw_x = InputStream.of(driver::getLeftY).log("/Robot/raw x").negate();
    InputStream raw_y = InputStream.of(driver::getLeftX).log("/Robot/raw y").negate();
    // Apply speed multiplier, deadband, square inputs, and scale translation to max speed
    InputStream r =
        InputStream.hypot(raw_x, raw_y)
            .scale(() -> speedMultiplier)
            .clamp(1.0)
            .deadband(Constants.DEADBAND, 1.0)
            .signedPow(2.0)
            .scale(MAX_SPEED.in(MetersPerSecond));

    InputStream theta = InputStream.atan(raw_x, raw_y);

    // Split x and y components of translation input
    InputStream x =
        r.scale(theta.map(Math::cos))
            .log("/Robot/final x"); // .rateLimit(MAX_ACCEL.in(MetersPerSecondPerSecond));
    InputStream y =
        r.scale(theta.map(Math::sin))
            .log("/Robot/final y"); // .rateLimit(MAX_ACCEL.in(MetersPerSecondPerSecond));

    // Apply speed multiplier, deadband, square inputs, and scale rotation to max teleop speed
    InputStream omega =
        InputStream.of(driver::getRightX)
            .negate()
            .scale(() -> speedMultiplier)
            .clamp(1.0)
            .deadband(DEADBAND, 1.0)
            .signedPow(2.0)
            .scale(TELEOP_ANGULAR_SPEED.in(RadiansPerSecond))
            .rateLimit(MAX_ANGULAR_ACCEL.in(RadiansPerSecond.per(Second)));

    drive.setDefaultCommand(drive.drive(x, y, omega, elevator::position).withName("joysticks"));

    scoral.blocked.onTrue(rumble(RumbleType.kBothRumble, 0.5));
    hopper.blocked.onFalse(rumble(RumbleType.kBothRumble, 0.5));

    disabled()
        .onTrue(
            Commands.runOnce(
                () -> vision.setPoseStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)))
        .onFalse(
            Commands.runOnce(
                () -> {
                  //   drive
                  //       .resetGyro(allianceRotation().plus(drive.heading()))
                  //       .withName("gyro reset enabled");
                  vision.setPoseStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                }));

    autonomous().whileTrue(Commands.deferredProxy(autos::getSelected).alongWith(leds.autos()));
    if (TUNING) {
      SignalLogger.enableAutoLogging(false);

      // manual .start() call is blocking, for up to 100ms
      teleop().onTrue(Commands.runOnce(() -> SignalLogger.start()));
      disabled().onTrue(Commands.runOnce(() -> SignalLogger.stop()));
    }

    test().whileTrue(systemsCheck());

    // Dashboard.cameraFL()
    //     .onTrue(
    //         Commands.runOnce(() -> vision.enableCam(FRONT_LEFT_CAMERA.name()))
    //             .ignoringDisable(true))
    //     .onFalse(
    //         Commands.runOnce(() -> vision.disableCam(FRONT_LEFT_CAMERA.name()))
    //             .ignoringDisable(true));
    // Dashboard.cameraFR()
    //     .onTrue(
    //         Commands.runOnce(() -> vision.enableCam(FRONT_RIGHT_CAMERA.name()))
    //             .ignoringDisable(true))
    //     .onFalse(
    //         Commands.runOnce(() -> vision.disableCam(FRONT_RIGHT_CAMERA.name()))
    //             .ignoringDisable(true));

    // DRIVER
    driver
        .leftBumper()
        .or(driver.rightBumper())
        .onTrue(Commands.runOnce(() -> speedMultiplier = Constants.SLOW_SPEED_MULTIPLIER))
        .onFalse(Commands.runOnce(() -> speedMultiplier = Constants.FULL_SPEED_MULTIPLIER));

    // RT to intake, LT to run backwards
    driver.rightTrigger().whileTrue(scoraling.hpsIntake());

    driver.a().whileTrue(align.source().withDeadline(scoraling.hpsIntake().asProxy()));

    driver.x().and(driver.povDown()).whileTrue(align.nearReef(Side.LEFT, Level.L2));
    driver.b().and(driver.povDown()).whileTrue(align.nearReef(Side.RIGHT, Level.L2));

    driver.x().and(driver.povDown().negate()).whileTrue(align.nearReef(Side.LEFT, Level.L4));
    driver.b().and(driver.povDown().negate()).whileTrue(align.nearReef(Side.RIGHT, Level.L4));

    driver.y().whileTrue(align.barge());

    driver.povLeft().onTrue(drive.zeroHeading());

    driver.povDown().whileTrue(elevator.homingSequence());

    driver.povUp().whileTrue(align.nearAlgae());

    // OPERATOR

    //
    // operator.rightTrigger()

    operator.leftBumper().whileTrue(scoral.score());
    operator.rightBumper().whileTrue(scoral.expalgae());

    operator
        .a()
        .whileTrue(
            elevator
                .scoreLevel(Level.L2_ALGAE)
                .asProxy()
                .alongWith(
                    Commands.waitUntil(elevator::atGoal).andThen(scoral.stealgae().asProxy())));

    operator
        .x()
        .whileTrue(
            elevator
                .scoreLevel(Level.L3_ALGAE)
                .asProxy()
                .alongWith(
                    Commands.waitUntil(elevator::atGoal).andThen(scoral.stealgae().asProxy())));

    // corolling
    operator
        .rightTrigger()
        .and(operator.b())
        .whileTrue(corolling.algaeIntake())
        .onFalse(corolling.processorGoTo());
    operator.leftTrigger().and(operator.b()).whileTrue(corolling.processorOuttake());

    operator
        .rightTrigger()
        .and(operator.b().negate())
        .whileTrue(corolling.coralIntake())
        .onFalse(coroller.coralIntake());
    operator.leftTrigger().and(operator.b().negate()).whileTrue(corolling.trough());

    // operator.b().toggleOnTrue(arm.manualArm(operator::getLeftY));

    operator.y().whileTrue(scoraling.runRollersBack());

    operator
        .povRight()
        .whileTrue(
            elevator
                .scoreLevel(Level.L2)
                .alongWith(
                    leds.progressGradient(
                        () -> 1 - elevator.position() / Level.L2.extension.in(Meters),
                        elevator::atGoal)));
    operator
        .povUp()
        .whileTrue(
            elevator
                .scoreLevel(Level.L3)
                .alongWith(
                    leds.progressGradient(
                        () -> 1 - elevator.position() / Level.L3.extension.in(Meters),
                        elevator::atGoal)));

    operator
        .povLeft()
        .whileTrue(
            elevator
                .scoreLevel(Level.L4)
                .alongWith(
                    leds.progressGradient(
                        () -> 1 - elevator.position() / Level.L4.extension.in(Meters),
                        elevator::atGoal)));
    operator.povDown().whileTrue(scoraling.noElevatorIntake());

    // DASHBOARD
    // TO REEF - DASHBOARD SELECT + DRIVER A
    // Dashboard.reef()
    //     .and(driver.y())
    //     .whileTrue(
    //         Commands.deferredProxy(
    //                 () -> align.reef(Dashboard.getLevelEntry(), Dashboard.getBranchEntry()))
    //             .alongWith(leds.blink(Color.kAqua)));

    Dashboard.elevator().whileTrue(elevator.goTo(() -> Dashboard.getElevatorEntry()));

    scoral.blocked.onFalse(leds.blink(Color.kLime));
  }

  @Logged
  public boolean isBlueAlliance() {
    return alliance() == Alliance.Blue;
  }

  /**
   * Command factory to make both controllers rumble.
   *
   * @param rumbleType The area of the controller to rumble.
   * @param strength The intensity of the rumble.
   * @return The command to rumble both controllers.
   */
  public Command rumble(RumbleType rumbleType, double strength) {
    return Commands.runOnce(
            () -> {
              driver.getHID().setRumble(rumbleType, strength);
              operator.getHID().setRumble(rumbleType, strength);
            })
        .andThen(Commands.waitSeconds(0.3))
        .finallyDo(
            () -> {
              driver.getHID().setRumble(rumbleType, 0);
              operator.getHID().setRumble(rumbleType, 0);
            });
  }

  public Command systemsCheck() {
    return Test.toCommand(
            // Test.fromCommand(leds.blink(Color.kRed).withTimeout(0.5)),
            elevator.goToTest(Level.L1.extension),
            elevator.goToTest(ElevatorConstants.MIN_EXTENSION),
            scoraling.runRollersTest(),
            arm.goToTest(CORAL_INTAKE_ANGLE),
            arm.goToTest(TROUGH_OUTTAKE_ANGLE),
            Test.fromCommand(coroller.coralIntake().asProxy().withTimeout(0.5)),
            Test.fromCommand(coroller.coralOuttake().asProxy().withTimeout(0.5)),
            arm.goToTest(DEFAULT_ANGLE),
            drive.systemsCheck(),
            Test.fromCommand(
                scoral.scoreSlow().asProxy().until(scoral.blocked.negate()).withTimeout(1)),
            Test.fromCommand(
                Commands.run(
                        () ->
                            drive.setChassisSpeeds(
                                new ChassisSpeeds(1, 0, 0),
                                ControlMode.CLOSED_LOOP_VELOCITY,
                                ElevatorConstants.MIN_EXTENSION.in(Meters)))
                    .withTimeout(Seconds.of(0.1))))
        // Test.fromCommand(leds.solid(Color.kLime).withTimeout(0.5)))
        .withName("Test Mechanisms");
  }

  @Override
  public void close() {
    super.close();
    try {
      drive.close();
    } catch (Exception e) {
    }
  }
}
