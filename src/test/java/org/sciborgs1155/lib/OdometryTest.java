package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.drive.DriveConstants.STARTING_POSE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Random;
import java.util.stream.Stream;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.sciborgs1155.robot.drive.Drive;
import static org.sciborgs1155.lib.UnitTestingUtil.*;

public class OdometryTest {
  Random rand = new Random();

  Drive drive;
  BetterOdometry odometry;

  @BeforeEach
  public void setup() {
    drive = Drive.create();

    odometry =
        new BetterOdometry(drive::moduleStates, drive::heading, STARTING_POSE.getTranslation());
  }

  @Test
  public void moduleDisplacementTest() {
    Translation2d v0 =
        new Translation2d(rand.nextDouble() * 10, new Rotation2d(Math.PI * 2 * rand.nextDouble()));
    Translation2d v1 =
        new Translation2d(rand.nextDouble() * 10, new Rotation2d(Math.PI * 2 * rand.nextDouble()));

    Translation2d secant = BetterOdometry.moduleDisplacement(v0, v1);

    assertTrue(secant.getNorm() < (v0.getNorm() + v1.getNorm()) * PERIOD.in(Seconds));

    assertTrue(v0.getY() + v1.getY() > 0 == secant.getY() > 0);
    assertTrue(v0.getX() + v1.getX() > 0 == secant.getX() > 0);
  }

  @ParameterizedTest
  @MethodSource("driveInputs")
  public void driveDisplacementTest(double vx, double vy) {
    Pose2d start = drive.pose();

    run(drive.drive(() -> 1, () -> 1, () -> 0));

    assertTrue(vx > 0 == drive.pose().getX() > start.getX());
    assertTrue(vy > 0 == drive.pose().getY() > start.getY());
  }

  private static Stream<Arguments> driveInputs() {
    return Stream.of(Arguments.of(1, 1),
    Arguments.of(-1, 1),
    Arguments.of(-1, -1),
    Arguments.of(1,-1));
  }
}
