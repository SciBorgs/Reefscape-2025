package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.drive.DriveConstants.STARTING_POSE;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Random;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.drive.Drive;

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

    assert (secant.getNorm() < (v0.getNorm() + v1.getNorm()) * PERIOD.in(Seconds));
  }
}
