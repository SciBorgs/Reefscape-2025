package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.NoGyro;
import org.sciborgs1155.robot.drive.SimModule;

/** Swerve test. Currently incomplete and does nothing. */
public class SwerveTest {
  SimModule frontLeft;
  SimModule frontRight;
  SimModule rearLeft;
  SimModule rearRight;
  NoGyro gyro;
  Drive drive;

  final double DELTA = 0.15;

  @BeforeEach
  public void setup() {
    setupTests();
    frontLeft = new SimModule("FL");
    frontRight = new SimModule("FR");
    rearLeft = new SimModule("RL");
    rearRight = new SimModule("RR");
    gyro = new NoGyro();
    drive = new Drive(gyro, frontLeft, frontRight, rearLeft, rearRight);
    drive.resetEncoders();
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(drive);
  }

  @Test
  public void systemCheck() {
    runUnitTest(drive.systemsCheck());
  }

  @RepeatedTest(5)
  public void reachesRobotVelocity() {
    double xVelocitySetpoint = Math.random() * (2 * 2.265) - 2.265;
    double yVelocitySetpoint = Math.random() * (2 * 2.265) - 2.265;
    run(
        drive.run(
            () ->
                drive.setChassisSpeeds(
                    new ChassisSpeeds(xVelocitySetpoint, yVelocitySetpoint, 0),
                    ControlMode.CLOSED_LOOP_VELOCITY)));
    run(drive.drive(() -> xVelocitySetpoint, () -> yVelocitySetpoint, drive::heading));
    fastForward(500);

    ChassisSpeeds chassisSpeed = drive.fieldRelativeChassisSpeeds();

    assertEquals(xVelocitySetpoint, chassisSpeed.vxMetersPerSecond, DELTA);
    assertEquals(yVelocitySetpoint, chassisSpeed.vyMetersPerSecond, DELTA);
  }

  @RepeatedTest(5)
  public void reachesAngularVelocity() {
    double omegaRadiansPerSecond = Math.random() * 2 - 1;
    run(
        drive.run(
            () ->
                drive.setChassisSpeeds(
                    new ChassisSpeeds(0, 0, omegaRadiansPerSecond),
                    ControlMode.CLOSED_LOOP_VELOCITY)));
    fastForward();

    ChassisSpeeds chassisSpeed = drive.robotRelativeChassisSpeeds();
    assertEquals(omegaRadiansPerSecond, chassisSpeed.omegaRadiansPerSecond, DELTA);
  }

  @RepeatedTest(5)
  public void testModuleDistance() {
    double xVelocitySetpoint = Math.random() * (2 * 2.265) - 2.265;
    double yVelocitySetpoint = Math.random() * (2 * 2.265) - 2.265;

    double deltaT = 4;
    double deltaX = xVelocitySetpoint * deltaT;
    double deltaY = yVelocitySetpoint * deltaT;
    run(
        drive.run(
            () ->
                drive.setChassisSpeeds(
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                        xVelocitySetpoint, yVelocitySetpoint, 0, drive.heading()),
                    ControlMode.CLOSED_LOOP_VELOCITY)));

    fastForward(Seconds.of(deltaT));

    Pose2d pose = drive.pose();

    assertEquals(deltaX, pose.getX(), DELTA * 2);
    assertEquals(deltaY, pose.getY(), DELTA * 2);
  }
}
