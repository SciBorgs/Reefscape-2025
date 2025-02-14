package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.runToCompletion;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.arm.ArmConstants.DEFAULT_ANGLE;
import static org.sciborgs1155.robot.arm.ArmConstants.INTAKE_ANGLE;
import static org.sciborgs1155.robot.arm.ArmConstants.PROCESSOR_OUTTAKE_ANGLE;
import static org.sciborgs1155.robot.arm.ArmConstants.TROUGH_OUTTAKE_ANGLE;

import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.arm.Arm;
import org.sciborgs1155.robot.arm.ArmConstants;
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

  @Test
  public void processorTest() {
    Command testcmd = corolling.processor();
    runToCompletion(testcmd);
    assertEquals(
        arm.position(),
        PROCESSOR_OUTTAKE_ANGLE.in(Radians),
        ArmConstants.POSITION_TOLERANCE.in(Radians));
    assertTrue(testcmd::isFinished);

    assertTrue(arm.getDefaultCommand().equals(arm.getCurrentCommand()));
    fastForward();
    assertEquals(
        arm.position(), DEFAULT_ANGLE.in(Radians), ArmConstants.POSITION_TOLERANCE.in(Radians));
  }

  @Test
  public void intakeTest() {
    Command testcmd = corolling.intake();
    runToCompletion(testcmd);
    assertEquals(
        arm.position(), INTAKE_ANGLE.in(Radians), ArmConstants.POSITION_TOLERANCE.in(Radians));
    assertTrue(testcmd::isFinished);

    assertTrue(arm.getDefaultCommand().equals(arm.getCurrentCommand()));
    fastForward();
    assertEquals(
        arm.position(), DEFAULT_ANGLE.in(Radians), ArmConstants.POSITION_TOLERANCE.in(Radians));
  }

  @Test
  public void troughTest() {
    Command testcmd = corolling.trough();
    runToCompletion(testcmd);
    assertEquals(
        arm.position(),
        TROUGH_OUTTAKE_ANGLE.in(Radians),
        ArmConstants.POSITION_TOLERANCE.in(Radians));
    assertTrue(testcmd::isFinished);
    assertTrue(arm.getDefaultCommand().equals(arm.getCurrentCommand()));
    fastForward();
    assertEquals(
        arm.position(), DEFAULT_ANGLE.in(Radians), ArmConstants.POSITION_TOLERANCE.in(Radians));
  }
}
