package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.run;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.Constants.allianceReflect;
import static org.sciborgs1155.robot.Constants.Field.CENTER_REEF;
import static org.sciborgs1155.robot.Constants.Field.LENGTH;
import static org.sciborgs1155.robot.Constants.Field.WIDTH;

import java.util.Random;
import java.util.stream.Stream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.sciborgs1155.robot.Constants.Field;
import org.sciborgs1155.robot.Constants.Field.Branch;
import org.sciborgs1155.robot.commands.Alignment;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.scoral.Scoral;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class AlignTest {

  private final Distance DELTA = Centimeters.of(3);
  private Alignment align;

  private Drive drive;

  private Elevator elevator;
  private Scoral scoral;

  @BeforeEach
  public void setup() {
    setupTests();
    this.drive = Drive.create();
    elevator = Elevator.create();
    scoral = Scoral.create();
    drive.resetEncoders();

    align = new Alignment(drive, elevator, scoral);
  }

  @AfterEach
  public void destroy() throws Exception {
    reset();
  }

  @RepeatedTest(5)
  void reflectionTest() throws Exception {
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
  void pathfindTest(Pose2d goal) {
    Pose2d gaming = new Pose2d(LENGTH.times(3/4), WIDTH.times(3/4), new Rotation2d());

    drive.resetOdometry(new Pose2d(Meters.of(1), WIDTH.div(2), Rotation2d.fromRadians(0)));
    run(align.pathfind(gaming));
    System.out.println(drive.toString());
    fastForward(Seconds.of(1));
    System.out.println(drive.toString());
    fastForward(Seconds.of(1));
    System.out.println(drive.toString());
    fastForward(Seconds.of(1));
    System.out.println(drive.toString());
    fastForward(Seconds.of(1));
    System.out.println(drive.toString());

    assertEquals(gaming.getX(), drive.pose().getX(), DELTA.in(Meters));
    assertEquals(gaming.getY(), drive.pose().getY(), DELTA.in(Meters));
    assertEquals(gaming.getRotation().getRadians(), drive.pose().getRotation().getRadians(), 0.1);
  }

  @Test
  void obstacleTest() {
    align.pathfind(new Pose2d(CENTER_REEF, Rotation2d.fromDegrees(0)));
    fastForward(Seconds.of(10));
    assertNotEquals(CENTER_REEF.getX(), drive.pose().getX(), DELTA.in(Meters));
    assertNotEquals(CENTER_REEF.getY(), drive.pose().getY(), DELTA.in(Meters));
  }

  private static Stream<Arguments> pathGoals() {
    return Stream.of(
        Arguments.of(
            /*Branch.B.pose,
            Branch.A.pose,
            Branch.L.pose,
            Branch.D.pose,
            Branch.G.pose,
            Branch.H.pose*/
             new Pose2d(LENGTH.times(3/4), WIDTH.times(3/4), new Rotation2d())));
  }
}
