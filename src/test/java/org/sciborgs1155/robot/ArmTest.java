package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.*;
import static org.sciborgs1155.robot.arm.ArmConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.arm.ArmConstants.MIN_ANGLE;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
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

  @Test
  public void fullExtension() {
    runUnitTest(arm.goToTest(MIN_ANGLE));
    runUnitTest(arm.goToTest(MAX_ANGLE));
  }

  @RepeatedTest(5)
  public void randExtension() {
    runUnitTest(
        arm.goToTest(
            Radians.of(Math.random() * (MAX_ANGLE.minus(MIN_ANGLE).in(Radians))).plus(MIN_ANGLE)));
  }
}
