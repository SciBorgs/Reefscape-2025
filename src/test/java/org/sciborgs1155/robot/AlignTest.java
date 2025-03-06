package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.runToCompletion;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.Constants.allianceReflect;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.util.Arrays;
import java.util.Random;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.sciborgs1155.robot.Constants.Field;
import org.sciborgs1155.robot.commands.Alignment;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.scoral.Scoral;

public class AlignTest {

  Drive drive;
  Elevator elevator;
  Scoral scoral;
  Alignment align;

  @BeforeEach
  public void setup() {
    setupTests();
    drive = Drive.create();
    elevator = Elevator.create();
    scoral = Scoral.create();
    drive.resetEncoders();
    drive.resetOdometry(new Pose2d());
    align = new Alignment(drive, elevator, scoral);
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(drive, elevator, scoral);
  }

  @RepeatedTest(5)
  public void reflectionTest() {
    Random rand = new Random();
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    Pose2d bluePose =
        new Pose2d(
            Field.LENGTH.times(rand.nextDouble()),
            Field.WIDTH.times(rand.nextDouble()),
            Rotation2d.fromRotations(rand.nextDouble()));
    Pose2d redPose = allianceReflect(bluePose);

    assertEquals(Field.LENGTH.minus(bluePose.getMeasureX()), redPose.getMeasureX());
    assertEquals(Field.WIDTH.minus(bluePose.getMeasureY()), redPose.getMeasureY());
    assertEquals(
        bluePose.getRotation().rotateBy(Rotation2d.fromRotations(0.5)), redPose.getRotation());
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  /** Tests whether the obstacle-avoiding pathing works correctly. */
  @ParameterizedTest
  @MethodSource("goals")
  public void pathfindTest(Pose2d pose) throws Exception {
    // Make and run the pathfinding command
    runToCompletion(align.alignTo(pose).withTimeout(Seconds.of(20)));

    // Assert the command works
    assertEquals(pose.getX(), drive.pose().getX(), Translation.TOLERANCE.in(Meters));
    assertEquals(pose.getY(), drive.pose().getY(), Translation.TOLERANCE.in(Meters));
    assertEquals(
        0,
        pose.getRotation().minus(drive.pose().getRotation()).getRadians(),
        Rotation.TOLERANCE.in(Radians));
  }

  private static Stream<Arguments> goals() {
    return Arrays.stream(Constants.Field.Branch.values()).map(v -> Arguments.of(v.pose));
  }
}
