package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.*;

import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.sciborgs1155.robot.Constants.Field.Level;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.SimElevator;

public class ElevatorTest {
  private Elevator elevator;
  private final double DELTA = 0.05;

  @BeforeEach
  public void initialize() {
    setupTests();
    elevator = new Elevator(new SimElevator());
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(elevator);
  }

  @ParameterizedTest
  @MethodSource("providePositionValues")
  public void reachesPosition(Level level) {
    run(elevator.scoreLevel(level), 200);
    assertEquals(level.getHeight(), elevator.position(), DELTA);
  }

  private static Stream<Arguments> providePositionValues() {
    return Stream.of(
        Arguments.of(Level.L1),
        Arguments.of(Level.L2),
        Arguments.of(Level.L3),
        Arguments.of(Level.L4));
  }
}
