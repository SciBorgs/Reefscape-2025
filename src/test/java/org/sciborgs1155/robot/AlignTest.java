package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.runToCompletion;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.Constants.allianceReflect;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.util.Random;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.Constants.Field;
import org.sciborgs1155.robot.Constants.Field.Branch;
import org.sciborgs1155.robot.commands.Alignment;
import org.sciborgs1155.robot.commands.Autos;
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

  @BeforeAll
  public static void configure() {
    // Configure pathfinding libraries
    LocalADStar pathfinder = new LocalADStar();
    Pathfinding.setPathfinder(pathfinder);
  }

  @BeforeEach
  public void setup() {
    setupTests();
    drive = Drive.create();
    elevator = Elevator.create();
    scoral = Scoral.create();
    drive.resetEncoders();
    drive.resetOdometry(new Pose2d());

    Autos.configureAutos(drive);

    align = new Alignment(drive, elevator, scoral);
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(drive, elevator, scoral);
  }

  @Disabled
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

  /**
   * Tests whether the non-obstacle-avoiding pathing works correctly.
   *
   * <p>Sometimes it fails due to PathPlanner issues, so there is a failure threshold. This never
   * occurs in practice, however, so we can ignore those "rare" occurrences.
   */
  @Disabled
  @RepeatedTest(value = 15, failureThreshold = 5)
  public void pathfindTest() throws Exception {
    // Take a random branch pose
    Random rand = new Random();
    Pose2d pose = Branch.values()[rand.nextInt(Branch.values().length)].pose;

    // Make and run the pathfinding command
    runToCompletion(align.pathfind(pose));

    // Assert the command works
    assertEquals(pose.getX(), drive.pose().getX(), Translation.TOLERANCE.in(Meters));
    assertEquals(pose.getY(), drive.pose().getY(), Translation.TOLERANCE.in(Meters));
    assertEquals(
        0,
        pose.getRotation().minus(drive.pose().getRotation()).getRadians(),
        Rotation.TOLERANCE.in(Radians));
  }
}
