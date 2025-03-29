package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.runToCompletion;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.drive.DriveConstants.driveSim;
import static org.sciborgs1155.robot.drive.DriveConstants.driveSimAdded;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Arrays;
import java.util.stream.Stream;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.sciborgs1155.robot.FieldConstants.Branch;
import org.sciborgs1155.robot.commands.Alignment;
import org.sciborgs1155.robot.commands.Scoraling;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.led.LEDs;
import org.sciborgs1155.robot.scoral.Scoral;

public class AlignTest {
  Drive drive;
  Elevator elevator;
  Scoral scoral;
  Hopper hopper;
  Alignment align;
  Scoraling scoraling;
  LEDs leds;

  @BeforeEach
  public void setup() {
    setupTests();
    drive = Drive.create();
    elevator = Elevator.create();
    scoral = Scoral.create();
    hopper = Hopper.create();
    leds = LEDs.create();
    scoraling = new Scoraling(hopper, scoral, elevator, leds);

    if (!driveSimAdded) {
      SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
      driveSimAdded = true;
    }

    drive.resetEncoders();
    drive.resetOdometry(new Pose2d());
    align = new Alignment(drive, elevator, scoral, scoraling, leds);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(drive, elevator, scoral);
  }

  /** Tests whether the obstacle-avoiding pathing works correctly. */
  @ParameterizedTest
  @MethodSource("goals")
  public void pathfindTest(Branch branch) throws Exception {
    Pose2d pose = branch.pose();
    // Make and run the pathfinding command
    runToCompletion(
        align
            .alignTo(() -> pose)
            .withTimeout(Seconds.of(75))
            .deadlineFor(Commands.run(() -> SimulatedArena.getInstance().simulationPeriodic())));

    // Assert the command works
    assertEquals(pose.getX(), drive.pose().getX(), Translation.TOLERANCE.in(Meters) * 5);
    assertEquals(pose.getY(), drive.pose().getY(), Translation.TOLERANCE.in(Meters) * 5);
    assertEquals(
        0,
        pose.getRotation().minus(drive.pose().getRotation()).getRadians(),
        Rotation.TOLERANCE.in(Radians));
  }

  private static Stream<Arguments> goals() {
    return Arrays.stream(FieldConstants.Branch.values()).map(v -> Arguments.of(v));
  }
}
