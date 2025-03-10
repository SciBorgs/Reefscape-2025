package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.FieldConstants.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.util.Random;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;

public class CalculationsTest {
  Random rand = new Random();

  @RepeatedTest(5)
  public void reflectionTest() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    Pose2d bluePose =
        new Pose2d(
            LENGTH.times(rand.nextDouble()),
            WIDTH.times(rand.nextDouble()),
            Rotation2d.fromRotations(rand.nextDouble()));
    Pose2d redPose = allianceReflect(bluePose);

    assertEquals(LENGTH.minus(bluePose.getMeasureX()), redPose.getMeasureX());
    assertEquals(WIDTH.minus(bluePose.getMeasureY()), redPose.getMeasureY());
    assertEquals(
        bluePose.getRotation().rotateBy(Rotation2d.fromRotations(0.5)), redPose.getRotation());
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  @RepeatedTest(5)
  public void advanceTest() {
    Distance distance = Meters.of(rand.nextDouble() * 10 - 5);
    Pose2d pose =
        new Pose2d(
            LENGTH.times(rand.nextDouble()),
            WIDTH.times(rand.nextDouble()),
            Rotation2d.fromRotations(rand.nextDouble()));
    Pose2d actual = pose.transformBy(advance(distance));

    Translation2d translation = new Translation2d(distance.in(Meters), pose.getRotation());
    Pose2d expected = new Pose2d(pose.getTranslation().plus(translation), pose.getRotation());

    assertEquals(expected.getX(), actual.getX(), Translation.TOLERANCE.in(Meters));
    assertEquals(expected.getY(), actual.getY(), Translation.TOLERANCE.in(Meters));
    assertEquals(
        expected.getRotation().minus(actual.getRotation()).getRadians(),
        0,
        Rotation.TOLERANCE.in(Radians));
  }

  @RepeatedTest(5)
  public void strafeTest() {
    Distance distance = Meters.of(rand.nextDouble() * 10 - 5);
    Pose2d pose =
        new Pose2d(
            LENGTH.times(rand.nextDouble()),
            WIDTH.times(rand.nextDouble()),
            Rotation2d.fromRotations(rand.nextDouble()));
    Pose2d actual = pose.transformBy(strafe(distance));

    Translation2d translation =
        new Translation2d(
            distance.in(Meters), pose.getRotation().plus(Rotation2d.fromDegrees(-90)));
    Pose2d expected = new Pose2d(pose.getTranslation().plus(translation), pose.getRotation());

    assertEquals(expected.getX(), actual.getX(), Translation.TOLERANCE.in(Meters));
    assertEquals(expected.getY(), actual.getY(), Translation.TOLERANCE.in(Meters));
    assertEquals(
        expected.getRotation().minus(actual.getRotation()).getRadians(),
        0,
        Rotation.TOLERANCE.in(Radians));
  }
}
