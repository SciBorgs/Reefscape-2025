package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.run;
import static org.sciborgs1155.lib.UnitTestingUtil.runToCompletion;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.Constants.Field.Branch.*;
import static org.sciborgs1155.robot.Constants.allianceReflect;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Random;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
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

  private Alignment align;

  private Drive drive;

  private Elevator elevator;
  private Scoral scoral;

  @BeforeEach
  public void setup() {
    setupTests();
    this.drive = Drive.create();
    this.elevator = Elevator.create();
    this.scoral = Scoral.create();
    this.drive.resetEncoders();

    Autos.configureAutos(drive);
    Pathfinding.setPathfinder(new LocalADStar());

    align = new Alignment(drive, elevator, scoral);
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(drive, elevator, scoral);
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

  @ParameterizedTest()
  @MethodSource("pathGoals")
  public void pathfindTest(Branch branch) {
    // Command testcmd = align.reef(Level.L4, branch);
    Command testcmd = align.pathfind(branch.pose);
    runToCompletion(testcmd);
    assertEquals(branch.pose.getX(), drive.pose().getX(), Translation.TOLERANCE.in(Meters));
    assertEquals(branch.pose.getY(), drive.pose().getY(), Translation.TOLERANCE.in(Meters));
    assertEquals(
        0,
        branch.pose.getRotation().minus(drive.pose().getRotation()).getRadians(),
        Rotation.TOLERANCE.in(Radian));
  }

  private static Stream<Arguments> pathGoals() {
    return Stream.of(
        Arguments.of(A),
        Arguments.of(B),
        Arguments.of(C),
        Arguments.of(D),
        Arguments.of(E),
        Arguments.of(F),
        Arguments.of(G),
        Arguments.of(H),
        Arguments.of(I),
        Arguments.of(J),
        Arguments.of(K),
        Arguments.of(L));
  }
}
