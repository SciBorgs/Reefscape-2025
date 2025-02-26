package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static org.sciborgs1155.robot.Constants.DEADBAND;
import static org.sciborgs1155.robot.Constants.Field.Branch.*;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Constants.ROBOT_TYPE;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
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
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.arm.Arm;
import org.sciborgs1155.robot.commands.Alignment;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.commands.Scoraling;
import org.sciborgs1155.robot.coroller.Coroller;
import org.sciborgs1155.robot.commands.Dashboard;
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
        default -> new Vision();
      };

  private final Elevator elevator =
      switch (ROBOT_TYPE) {
        case FULL, SCORALING -> Elevator.create();
        default -> Elevator.none();
      };

  private final Scoral scoral =
      switch (ROBOT_TYPE) {
        case FULL, SCORALING -> Scoral.create();
        default -> Scoral.none();
      };

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
  private final LEDStrip rightLED = new LEDStrip(60, 119, true);

  private final Scoraling scoraling = new Scoraling(hopper, scoral, elevator, leftLED, rightLED);

  // COMMANDS
  @Log.NT private final SendableChooser<Command> autos = Autos.configureAutos(drive);
  @Log.NT private final Alignment align = new Alignment(drive, elevator, scoral);

  @Log.NT private double speedMultiplier = Constants.FULL_SPEED_MULTIPLIER;

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    super(PERIOD.in(Seconds));
    configureGameBehavior();
    configureBindings();
  }

  /** Configures basic behavior for different periods during the game. */
  private void configureGameBehavior() {
    // Configure logging with DataLogManager, Monologue, URCL, and FaultLogger
    DataLogManager.start();
    Monologue.setupMonologue(this, "/Robot", false, true);
    Dashboard.configure();
    addPeriodic(Monologue::updateAll, PERIOD.in(Seconds));
    addPeriodic(FaultLogger::update, 2);

    SmartDashboard.putData(CommandScheduler.getInstance());
    // Log PDH
    SmartDashboard.putData("PDH", pdh);
    FaultLogger.register(pdh);

    // Configure pose estimation updates from vision every tick
    addPeriodic(() -> drive.updateEstimates(vision.estimatedGlobalPoses()), PERIOD.in(Seconds));

    RobotController.setBrownoutVoltage(6.0);

    Pathfinding.setPathfinder(new LocalADStar());

    if (isReal()) {
      URCL.start(DataLogManager.getLog());
      pdh.clearStickyFaults();
      pdh.setSwitchableChannel(true);
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
      addPeriodic(() -> vision.simulationPeriodic(drive.pose()), PERIOD.in(Seconds));
    }

    addPeriodic(() -> Dashboard.tick(), PERIOD.in(Seconds));
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

    drive.setDefaultCommand(drive.drive(x, y, omega));

    driver.a().whileTrue(align.reef(Level.L3, A));
    driver.x().whileTrue(align.reef(Level.L4, D));
    driver.y().whileTrue(align.reef(Level.L4, G));

    // driver.b().whileTrue(align.pathfind(D.pose));

    driver.b().onTrue(drive.zeroHeading());

    // driver.x().whileTrue(drive.assistedDrive(x, y, omega, L.pose));
    // driver.y().whileTrue(drive.assistedDrive(x, y, omega, G.pose));

    // elevator.setDefaultCommand(elevator.retract());

    // leftLED.setDefaultCommand(leftLED.rainbow());
    // rightLED.setDefaultCommand(rightLED.rainbow());
    leftLED.setDefaultCommand(leftLED.music());
    middleLED.setDefaultCommand(middleLED.solid(Color.kYellow));
    rightLED.setDefaultCommand(rightLED.music());

    teleop()
        .onTrue(
            leftLED
                .elevatorLED(() -> elevator.position() / ElevatorConstants.MAX_EXTENSION.in(Meters))
                .alongWith(
                    rightLED.elevatorLED(
                        () -> elevator.position() / ElevatorConstants.MAX_EXTENSION.in(Meters))));

    autonomous().whileTrue(Commands.deferredProxy(autos::getSelected));

    test().whileTrue(systemsCheck());
    // driver.b().whileTrue(drive.zeroHeading());
    driver
        .leftBumper()
        .or(driver.rightBumper())
        .onTrue(Commands.runOnce(() -> speedMultiplier = Constants.SLOW_SPEED_MULTIPLIER))
        .onFalse(Commands.runOnce(() -> speedMultiplier = Constants.FULL_SPEED_MULTIPLIER));

    teleop().onTrue(Commands.runOnce(() -> SignalLogger.start()));
    disabled().onTrue(Commands.runOnce(() -> SignalLogger.stop()));

    operator.leftTrigger().whileTrue(elevator.scoreLevel(Level.L3_ALGAE));
    operator.leftBumper().whileTrue(scoral.score());
    operator.rightBumper().whileTrue(scoral.algae());
    operator.rightTrigger().onTrue(middleLED.blink(Color.kWhite));

    operator.a().onTrue(elevator.retract());
    operator.b().toggleOnTrue(elevator.manualElevator(InputStream.of(operator::getLeftY)));
    // operator.y().whileTrue(elevator.highFive());
    operator.x().whileTrue(scoraling.hpsIntake().alongWith(rumble(RumbleType.kBothRumble, 0.5)));
    operator.y().whileTrue(scoraling.runRollersBack());

    operator.povDown().onTrue(elevator.scoreLevel(Level.L1));
    operator.povRight().onTrue(elevator.scoreLevel(Level.L2));
    operator.povUp().whileTrue(scoraling.scoral(Level.L3));
    operator.povLeft().onTrue(elevator.scoreLevel(Level.L4));

    driver.povUp().onTrue(leftLED.rainbow());
    driver.povDown().onTrue(leftLED.music());
    driver.povLeft().onTrue(rightLED.blink(Color.kWhite));
    driver.povRight().onTrue(rightLED.scrolling());
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
            drive.systemsCheck(),
            elevator.goToTest(Level.L1.extension),
            Test.fromCommand(scoral.algae().withTimeout(2)))
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
