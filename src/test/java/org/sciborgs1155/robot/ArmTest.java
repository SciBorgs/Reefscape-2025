package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.*;
import static org.sciborgs1155.robot.arm.ArmConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.arm.ArmConstants.MIN_ANGLE;

import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
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

  /** Moves arm to a specific angle. */
  public void pointExtension(Angle angle) {
    Angle positionSetpoint = angle;
    System.out.println("Position Setpoint: " + positionSetpoint.toString());

    arm.goTowards(positionSetpoint);
    fastForward(500);

    assertEquals(positionSetpoint.in(Radians), arm.position(), TOLERANCE);
  }

  @Disabled
  @Test
  public void fullExtension() {
    pointExtension(MIN_ANGLE);
    pointExtension(MAX_ANGLE);
  }

  @Disabled
  @RepeatedTest(5)
  public void pointExtensionTest() {
    pointExtension(
        Radians.of(Math.random() * (MAX_ANGLE.minus(MIN_ANGLE).in(Radians))).plus(MIN_ANGLE));
  }
}
