package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.UnitTestingUtil.*;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.sciborgs1155.robot.commands.Scoraling;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.ElevatorConstants;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.led.LEDStrip;
import org.sciborgs1155.robot.scoral.Scoral;

public class ScoralingTest {
  Scoral scoral;
  Hopper hopper;
  Elevator elevator;
  Scoraling scoraling;
  LEDStrip leftLED;
  LEDStrip rightLED;

  @BeforeEach
  public void setup() {
    setupTests();
    scoral = Scoral.create();
    elevator = Elevator.create();
    hopper = Hopper.create();
    leftLED = new LEDStrip(0, 59, false);
    rightLED = new LEDStrip(60, 119, true);

    scoraling = new Scoraling(hopper, scoral, elevator, leftLED, rightLED);
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(scoral, elevator, hopper);
  }

  @Test
  public void hpsIntakeTest() {
    run(scoraling.hpsIntake());
    assertEquals(
        MIN_EXTENSION.in(Meters),
        elevator.position(),
        ElevatorConstants.POSITION_TOLERANCE.in(Meters));
    assertTrue(hopper.getCurrentCommand().getName() == "intakingHPS");
    assertTrue(scoral.getCurrentCommand().getName() == "intakingHPS");
  }

  @ParameterizedTest
  @MethodSource("levels")
  public void scoralTest(Level level) {
    run(scoraling.scoral(level));
    fastForward(Seconds.of(10));
    assertEquals(
        level.extension.in(Meters),
        elevator.position(),
        ElevatorConstants.POSITION_TOLERANCE.in(Meters) * 2);
    assertTrue(scoral.getCurrentCommand().getName() == "scoraling");
  }

  @Disabled // it works! but beambreak isn't simulated...
  @ParameterizedTest
  @MethodSource("levels")
  public void algaeCleanTest(Level level) {
    if (level == Level.L1 || level == Level.L4) {
      return;
    }
    run(scoraling.cleanAlgae(level));
    fastForward(Seconds.of(7));
    assertEquals(
        level.extension.in(Meters),
        elevator.position(),
        ElevatorConstants.POSITION_TOLERANCE.in(Meters) * 2);
    assertTrue(scoral.getCurrentCommand().getName() == "cleanAlgae");
  }

  private static Stream<Arguments> levels() {
    return Stream.of(
        Arguments.of(Level.L1),
        Arguments.of(Level.L2),
        Arguments.of(Level.L3),
        Arguments.of(Level.L4));
  }
}
