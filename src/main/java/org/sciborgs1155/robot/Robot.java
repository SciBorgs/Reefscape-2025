package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static org.sciborgs1155.robot.Constants.DEADBAND;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.ElevatorConstants;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
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
  private final Drive drive = Drive.create();
  private final Vision vision = Vision.create();
  private final Elevator elevator = Elevator.create();
  private final Scoral scoral = Scoral.create();

  private final LEDStrip led = new LEDStrip();

  // COMMANDS
  @Log.NT private final SendableChooser<Command> autos = Autos.configureAutos(drive);

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
    addPeriodic(Monologue::updateAll, PERIOD.in(Seconds));
    addPeriodic(FaultLogger::update, 2);

    SmartDashboard.putData(CommandScheduler.getInstance());
    // Log PDH
    SmartDashboard.putData("PDH", pdh);
    FaultLogger.register(pdh);

    // Configure pose estimation updates every tick
    addPeriodic(() -> drive.updateEstimates(vision.estimatedGlobalPoses()), PERIOD.in(Seconds));

    RobotController.setBrownoutVoltage(6.0);

    if (isReal()) {
      URCL.start(DataLogManager.getLog());
      pdh.clearStickyFaults();
      pdh.setSwitchableChannel(true);
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
      addPeriodic(() -> vision.simulationPeriodic(drive.pose()), PERIOD.in(Seconds));
    }
  }

  /** Configures trigger -> command bindings. */
  private void configureBindings() {
    InputStream x = InputStream.of(driver::getLeftX).log("raw x");
    InputStream y = InputStream.of(driver::getLeftY).log("raw y").negate();

    // Apply speed multiplier, deadband, square inputs, and scale translation to max speed
    InputStream r =
        InputStream.hypot(x, y)
            .log("Robot/raw joystick")
            .scale(() -> speedMultiplier)
            .clamp(1.0)
            .deadband(Constants.DEADBAND, 1.0)
            .signedPow(2.0)
            .log("Robot/processed joystick")
            .scale(MAX_SPEED.in(MetersPerSecond));

    InputStream theta = InputStream.atan(x, y);

    // Split x and y components of translation input
    x = r.scale(theta.map(Math::cos)); // .rateLimit(MAX_ACCEL.in(MetersPerSecondPerSecond));
    y = r.scale(theta.map(Math::sin)); // .rateLimit(MAX_ACCEL.in(MetersPerSecondPerSecond));

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
    elevator.setDefaultCommand(elevator.retract());
    led.setDefaultCommand(led.rainbow());
    led.elevatorLED(() -> elevator.position() / ElevatorConstants.MAX_EXTENSION.in(Meters));

    autonomous().whileTrue(Commands.deferredProxy(autos::getSelected));

    test().whileTrue(systemsCheck());
    driver.b().whileTrue(drive.zeroHeading());
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

    operator.a().onTrue(elevator.retract());
    operator.b().toggleOnTrue(elevator.manualElevator(InputStream.of(operator::getLeftY)));
    operator.y().whileTrue(elevator.highFive());

    operator.povDown().onTrue(elevator.scoreLevel(Level.L1));
    operator.povRight().onTrue(elevator.scoreLevel(Level.L2));
    operator.povUp().onTrue(elevator.scoreLevel(Level.L3));
    operator.povLeft().onTrue(elevator.scoreLevel(Level.L4));
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
