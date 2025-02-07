package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.Test.*;
import static org.sciborgs1155.lib.UnitTestingUtil.*;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.units.measure.Distance;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import org.sciborgs1155.robot.elevator.SimElevator;

public class ElevatorTest {
  private Elevator elevator;

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
  public void reachesPosition(Distance height) {
    runUnitTest(elevator.goToTest(height));
  }

  private static Stream<Arguments> providePositionValues() {
    return Stream.of(
        Arguments.of(MIN_HEIGHT),
        Arguments.of(Level.L1.extension),
        Arguments.of(Level.L2.extension),
        Arguments.of(Level.L3.extension),
        Arguments.of(Level.L4.extension),
        Arguments.of(MAX_HEIGHT));
  }
}
