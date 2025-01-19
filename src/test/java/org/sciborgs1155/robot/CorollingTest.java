package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.run;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.arm.Arm;
import org.sciborgs1155.robot.commands.Corolling;
import org.sciborgs1155.robot.coroller.Coroller;

public class CorollingTest {
  Arm arm;
  Coroller coroller;

  Corolling corolling;

  @BeforeEach
  public void setup() {
    setupTests();
    arm = Arm.create();
    coroller = Coroller.create();

    corolling = new Corolling(arm, coroller);
  }

  @AfterEach
  public void destroy() throws Exception {
    arm.close();
    coroller.close();
    reset();
  }

  /**
   * A test to determine if the intake command is scheduled and follows through, as well as
   * finishing.
   */
  public void finishes(Command testcmd) {
    run(testcmd);
    assert testcmd.isScheduled();
    assertFalse(arm.getCurrentCommand().equals(arm.getDefaultCommand()));
    assertFalse(coroller.getCurrentCommand().equals(coroller.getDefaultCommand()));
    fastForward();
    assertTrue(arm::atGoal);
    assertFalse(coroller.getCurrentCommand().equals(coroller.getDefaultCommand()));
    assertTrue(testcmd::isScheduled);
  }

  @Test
  public void finishTest() {
    finishes(corolling.intake());
    finishes(corolling.processor());
    finishes(corolling.trough());
  }
}
