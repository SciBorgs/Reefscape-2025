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
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_ANGULAR_ACCEL;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_SPEED;
import static org.sciborgs1155.robot.drive.DriveConstants.TELEOP_ANGULAR_SPEED;
import static org.sciborgs1155.robot.vision.VisionConstants.BACK_LEFT_CAMERA;
import static org.sciborgs1155.robot.vision.VisionConstants.BACK_MIDDLE_CAMERA;
import static org.sciborgs1155.robot.vision.VisionConstants.BACK_RIGHT_CAMERA;
import static org.sciborgs1155.robot.vision.VisionConstants.FRONT_LEFT_CAMERA;
import static org.sciborgs1155.robot.vision.VisionConstants.FRONT_RIGHT_CAMERA;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Arrays;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.FieldConstants.Face;
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
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.ElevatorConstants;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.led.LEDStrip;
import org.sciborgs1155.robot.scoral.Scoral;
import org.sciborgs1155.robot.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Logged {
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

  @IgnoreLogged
  private final Elevator elevator =
      switch (ROBOT_TYPE) {
        case FULL, SCORALING -> Elevator.create();
        default -> Elevator.none();
      };

  @IgnoreLogged
  private final Scoral scoral =
      switch (ROBOT_TYPE) {
        case FULL, SCORALING -> Scoral.create();
        default -> Scoral.none();
      };

  @IgnoreLogged
  private final Hopper hopper =
      switch (ROBOT_TYPE) {
        case FULL, SCORALING -> Hopper.create();
        default -> Hopper.none();
      };

  private final Coroller coroller =
      switch (ROBOT_TYPE) {
        case FULL, COROLLING -> Coroller.create();
        default -> Coroller.none();
      };

  private final Arm arm =
      switch (ROBOT_TYPE) {
        case FULL, COROLLING -> Arm.create();
        default -> Arm.none();
      };

  private final LEDStrip leftLED = new LEDStrip(0, 37, false);
  private final LEDStrip middleLED = new LEDStrip(38, 59, true);
  private final LEDStrip rightLED = new LEDStrip(60, 103, true);

  private final Scoraling scoraling = new Scoraling(hopper, scoral, elevator, leftLED, rightLED);
  private final Corolling corolling = new Corolling(arm, coroller);

  // COMMANDS
  @Log.NT private final Alignment align = new Alignment(drive, elevator, scoral);

  @Log.NT
  private final SendableChooser<Command> autos =
      Autos.configureAutos(drive, scoraling, elevator, align, scoral);

  @Log.NT private double speedMultiplier = Constants.FULL_SPEED_MULTIPLIER;

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    super(PERIOD.in(Seconds));
    configureGameBehavior();
    configureBindings();
  }

  /** Configures basic behavior for different periods during the game. */
  private void configureGameBehavior() {
    // Configure logging with DataLogManager, Monologue, and FaultLogger
    DataLogManager.start();
    Monologue.setupMonologue(this, "/Robot", false, true);
    addPeriodic(Monologue::updateAll, PERIOD.in(Seconds));
    addPeriodic(FaultLogger::update, 2);
    addPeriodic(vision::logCamEnabled, 1);

    SmartDashboard.putData(CommandScheduler.getInstance());
    // Log PDH
    SmartDashboard.putData("PDH", pdh);
    FaultLogger.register(pdh);

    if (TUNING) {
      addPeriodic(
          () ->
              log(
                  "camera transforms",
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
                      .toArray(Pose3d[]::new)),
          PERIOD.in(Seconds));
    }

    // Configure pose estimation updates from vision every tick
    addPeriodic(() -> drive.updateEstimates(vision.estimatedGlobalPoses()), PERIOD.in(Seconds));

    RobotController.setBrownoutVoltage(6.0);

    Pathfinding.setPathfinder(new LocalADStar());

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
    InputStream raw_x = InputStream.of(driver::getLeftY).log("raw x").negate();
    InputStream raw_y = InputStream.of(driver::getLeftX).log("raw y").negate();

    // Apply speed multiplier, deadband, square inputs, and scale translation to max speed
    InputStream r =
        InputStream.hypot(raw_x, raw_y)
            .log("Robot/raw joystick")
            .scale(() -> speedMultiplier)
            .clamp(1.0)
            .deadband(Constants.DEADBAND, 1.0)
            .signedPow(2.0)
            .log("Robot/processed joystick")
            .scale(MAX_SPEED.in(MetersPerSecond));

    InputStream theta = InputStream.atan(raw_x, raw_y);

    // Split x and y components of translation input
    InputStream x =
        r.scale(theta.map(Math::cos))
            .log("final x"); // .rateLimit(MAX_ACCEL.in(MetersPerSecondPerSecond));
    InputStream y =
        r.scale(theta.map(Math::sin))
            .log("final y"); // .rateLimit(MAX_ACCEL.in(MetersPerSecondPerSecond));

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

    drive.setDefaultCommand(drive.drive(x, y, omega).withName("joysticks"));

    // leftLED.setDefaultCommand(leftLED.rainbow());
    // middleLED.setDefaultCommand(middleLED.solid(Color.kYellow));
    // rightLED.setDefaultCommand(rightLED.rainbow());

    scoral.blocked.onTrue(rumble(RumbleType.kBothRumble, 0.5));
    hopper.blocked.onFalse(rumble(RumbleType.kBothRumble, 0.5));

    autonomous()
        .whileTrue(
            Commands.deferredProxy(autos::getSelected)
                .alongWith(leftLED.autos(), rightLED.autos(), middleLED.autos()));

    teleop().onTrue(Commands.runOnce(() -> SignalLogger.start()));

    disabled().onTrue(Commands.runOnce(() -> SignalLogger.stop()));

    test().whileTrue(systemsCheck());

    Dashboard.cameraFR()
        .onTrue(Commands.runOnce(() -> vision.enableCam(FRONT_RIGHT_CAMERA.name())))
        .onFalse(Commands.runOnce(() -> vision.disableCam(FRONT_RIGHT_CAMERA.name())));
    Dashboard.cameraFL()
        .onTrue(Commands.runOnce(() -> vision.enableCam(FRONT_LEFT_CAMERA.name())))
        .onFalse(Commands.runOnce(() -> vision.disableCam(FRONT_LEFT_CAMERA.name())));
    Dashboard.cameraBR()
        .onTrue(Commands.runOnce(() -> vision.enableCam(BACK_RIGHT_CAMERA.name())))
        .onFalse(Commands.runOnce(() -> vision.disableCam(BACK_RIGHT_CAMERA.name())));
    Dashboard.cameraBL()
        .onTrue(Commands.runOnce(() -> vision.enableCam(BACK_LEFT_CAMERA.name())))
        .onFalse(Commands.runOnce(() -> vision.disableCam(BACK_LEFT_CAMERA.name())));
    Dashboard.cameraBM()
        .onTrue(Commands.runOnce(() -> vision.enableCam(BACK_MIDDLE_CAMERA.name())))
        .onFalse(Commands.runOnce(() -> vision.disableCam(BACK_MIDDLE_CAMERA.name())));

    // DRIVER
    driver
        .leftBumper()
        .or(driver.rightBumper())
        .onTrue(Commands.runOnce(() -> speedMultiplier = Constants.SLOW_SPEED_MULTIPLIER))
        .onFalse(Commands.runOnce(() -> speedMultiplier = Constants.FULL_SPEED_MULTIPLIER));

    // RT to intake, LT to run backwards
    driver.rightTrigger().whileTrue(scoraling.hpsIntake());
    driver
        .a()
        .whileTrue(
            align
                .source()
                .alongWith(leftLED.blink(Color.kAqua).alongWith(rightLED.blink(Color.kAqua))));

    driver
        .x()
        .whileTrue(
            align
                .nearReef(Side.LEFT)
                .alongWith(
                    leftLED.progressGradient(
                        () ->
                            1
                                / (calculateAlignment(
                                    Face.nearest(drive.pose())
                                        .branch(Side.LEFT)
                                        .pose()
                                        .getTranslation()))),
                    rightLED.progressGradient(
                        () ->
                            1
                                / (calculateAlignment(
                                    Face.nearest(drive.pose())
                                        .branch(Side.LEFT)
                                        .pose()
                                        .getTranslation())))));

    driver
        .b()
        .whileTrue(
            align
                .nearReef(Side.RIGHT)
                .alongWith(
                    leftLED.progressGradient(
                        () ->
                            1
                                / (calculateAlignment(
                                    Face.nearest(drive.pose())
                                        .branch(Side.RIGHT)
                                        .pose()
                                        .getTranslation()))),
                    rightLED.progressGradient(
                        () ->
                            1
                                / (calculateAlignment(
                                    Face.nearest(drive.pose())
                                        .branch(Side.RIGHT)
                                        .pose()
                                        .getTranslation())))));

    // B for dashboard select
    driver.povLeft().onTrue(drive.zeroHeading());
    driver.povRight().whileTrue(corolling.intake());

    driver.povUp().whileTrue(coroller.intake());
    driver.povDown().whileTrue(coroller.outtake());

    // OPERATOR
    operator
        .leftTrigger()
        .whileTrue(
            elevator
                .scoreLevel(Level.L3_ALGAE)
                .alongWith(
                    leftLED.progressGradient(
                        () -> elevator.position() / ElevatorConstants.MAX_EXTENSION.in(Meters)),
                    rightLED.progressGradient(
                        () -> elevator.position() / ElevatorConstants.MAX_EXTENSION.in(Meters))));

    operator.rightTrigger().whileTrue(scoraling.hpsIntake());

    operator.leftBumper().whileTrue(scoral.score());
    operator.x().whileTrue(scoral.score(Level.L3));
    operator.rightBumper().whileTrue(scoral.algae());

    operator.b().toggleOnTrue(arm.manualArm(InputStream.of(operator::getLeftY)));
    operator.y().whileTrue(scoraling.runRollersBack());

    operator.povRight().whileTrue(elevator.scoreLevel(Level.L2));
    operator.povUp().whileTrue(elevator.scoreLevel(Level.L3));

    operator.povLeft().whileTrue(elevator.scoreLevel(Level.L4));
    operator.povDown().whileTrue(scoraling.noElevatorIntake());

    // DASHBOARD
    // TO REEF - DASHBOARD SELECT + DRIVER A
    Dashboard.reef()
        .and(driver.y())
        .whileTrue(
            Commands.deferredProxy(
                    () -> align.reef(Dashboard.getLevelEntry(), Dashboard.getBranchEntry()))
                .alongWith(
                    leftLED.blink(Color.kAqua),
                    rightLED.blink(Color.kAqua),
                    middleLED.solid(Color.kAqua)));

    Dashboard.elevator().whileTrue(elevator.goTo(() -> Dashboard.getElevatorEntry()));

    scoral.blocked.onFalse(leftLED.blink(Color.kLime).alongWith(rightLED.blink(Color.kLime)));
  }

  @Log.NT
  public boolean isBlueAlliance() {
    return alliance() == Alliance.Blue;
  }

  private double calculateAlignment(Translation2d target) {
    return drive.pose().getTranslation().minus(target).getNorm();
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
            Test.fromCommand(
                middleLED
                    .blink(Color.kRed)
                    .alongWith(leftLED.blink(Color.kRed), rightLED.blink(Color.kRed))
                    .withTimeout(0.5)),
            elevator.goToTest(Level.L1.extension),
            elevator.goToTest(ElevatorConstants.MIN_EXTENSION),
            scoraling.runRollersTest(),
            // arm.goToTest(INTAKE_ANGLE),
            // Test.fromCommand(coroller.outtake().withTimeout(1)),
            // Test.fromCommand(coroller.intake().withTimeout(1)),
            // arm.goToTest(DEFAULT_ANGLE),
            drive.systemsCheck(),
            Test.fromCommand(
                scoral.scoreSlow().asProxy().until(scoral.blocked.negate()).withTimeout(1)),
            Test.fromCommand(
                middleLED
                    .solid(Color.kLime)
                    .alongWith(leftLED.solid(Color.kLime), rightLED.solid(Color.kLime))
                    .withTimeout(0.5)))
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
