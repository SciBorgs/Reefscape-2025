package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.*;
import static org.sciborgs1155.robot.arm.ArmConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.arm.ArmConstants.MIN_ANGLE;

import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.robot.arm.Arm;

/** Tests {@link Arm} subsystem with simulated hardware. Doesn't work :/ */
public class ArmTest {
  Arm arm;

  final double TOLERANCE = 0.15;

  @BeforeEach
  public void setup() {
    setupTests();
    arm = Arm.create();
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(arm);
  }

  /** Moves arm to a specific angle. */
  public void pointExtension(Angle angle) {
    System.out.println("Position Setpoint: " + angle.toString());

    run(arm.goTo(angle));
    fastForward(500);

    assertEquals(angle.in(Radians), arm.position(), TOLERANCE);
  }

  @Test
  public void fullExtension() {
    pointExtension(MIN_ANGLE);
    pointExtension(MAX_ANGLE);
  }

  @RepeatedTest(5)
  public void pointExtensionTest() {
    runUnitTest(
        arm.goToTest(
            Radians.of(Math.random() * (MAX_ANGLE.minus(MIN_ANGLE).in(Radians))).plus(MIN_ANGLE)));
  }

  @ParameterizedTest
  @ValueSource(doubles = {2, 1, 0, -0.2})
  public void whjakfsd(double a) {
    runUnitTest(arm.goToTest(Radians.of(a)));
  }
}
