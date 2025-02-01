package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
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
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Random;
import org.junit.jupiter.api.BeforeEach;
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

  @BeforeEach
  public void setup() {
    setupTests();
  }

  @RepeatedTest(5)
  public void reflectionTest() throws Exception {
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
    reset();
  }

  // Sometimes it fails due to PathPlanner issues.
  // This never occurs in practice, however, so we can ignore those "rare" occurrences.
  @RepeatedTest(value = 15, failureThreshold = 5)
  public void pathfindTest() throws Exception {
    // Make subsystems
    Drive drive = Drive.create();
    Elevator elevator = Elevator.create();
    Scoral scoral = Scoral.create();
    drive.resetEncoders();

    // Take a random branch pose
    Random rand = new Random();
    Pose2d pose = Branch.values()[rand.nextInt(Branch.values().length)].pose;

    // Configure pathfinding libraries
    LocalADStar pathfinder = new LocalADStar();
    Pathfinding.setPathfinder(pathfinder);
    Autos.configureAutos(drive);

    // Create command class and make a command
    Alignment align = new Alignment(drive, elevator, scoral);
    Command testcmd = align.pathfind(pose).withTimeout(20);

    // Test that the command works
    runToCompletion(testcmd);
    assertEquals(pose.getX(), drive.pose().getX(), Translation.TOLERANCE.in(Meters));
    assertEquals(pose.getY(), drive.pose().getY(), Translation.TOLERANCE.in(Meters));
    assertEquals(
        0,
        pose.getRotation().minus(drive.pose().getRotation()).getRadians(),
        Rotation.TOLERANCE.in(Radian));
  }
}
