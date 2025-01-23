package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.run;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.Constants.allianceReflect;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Random;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.sciborgs1155.robot.Constants.Field;
import org.sciborgs1155.robot.Constants.Field.Branch;
import org.sciborgs1155.robot.Constants.Field.Level;
import org.sciborgs1155.robot.commands.Alignment;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.scoral.Scoral;

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
  void pathfindTest(Branch branch) {
    Command testcmd = align.reef(Level.L4, branch);
    run(testcmd);
    fastForward(Seconds.of(5));
    assertTrue(() -> !testcmd.isScheduled());
    assertEquals(branch.pose.getX(), drive.pose().getX(), Translation.TOLERANCE.in(Meters) * 3);
    assertEquals(branch.pose.getY(), drive.pose().getY(), Translation.TOLERANCE.in(Meters));
    assertEquals(
        branch.pose.getRotation().getRadians(),
        drive.pose().getRotation().getRadians(),
        Rotation.TOLERANCE.in(Radian) * 3);
  }

  private static Stream<Arguments> pathGoals() {
    return Stream.of(
        Arguments.of(
            Branch.A, Branch.B, Branch.C, Branch.D, Branch.E, Branch.F, Branch.G, Branch.H,
            Branch.I, Branch.J, Branch.K, Branch.L));
  }
}
