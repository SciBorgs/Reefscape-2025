package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radian;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.run;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.arm.Arm;
import org.sciborgs1155.robot.arm.ArmConstants;
import org.sciborgs1155.robot.commands.GroundIntake;
import org.sciborgs1155.robot.coroller.Coroller;

import edu.wpi.first.wpilibj2.command.Command;

public class GroundIntakeTest {
  Arm arm;
  Coroller coroller;

  GroundIntake intake;

  @BeforeEach
  public void setup() {
    setupTests();
    arm = Arm.create();
    coroller = Coroller.create();

    intake = new GroundIntake(arm, coroller);
  }

  @AfterEach
  public void destroy() throws Exception {
    arm.close();
    coroller.close();
    reset();
  }

  /**
   * A test to determine if the intake command is scheduled and follows through, as well as finishing.
   */
  @Test
  public void finishTest() {
    Command testcmd = intake.intake();
    run(testcmd);
    assert testcmd.isScheduled();
    assertFalse(arm.getCurrentCommand().equals(arm.getDefaultCommand()));
    assertFalse(coroller.getCurrentCommand().equals(coroller.getDefaultCommand()));
    fastForward();
    assertEquals(arm.position(), ArmConstants.INTAKE_ANGLE.in(Radian), 0.02);
    assertFalse(coroller.getCurrentCommand().equals(coroller.getDefaultCommand()));
    assertTrue(testcmd::isScheduled);
  }
}
