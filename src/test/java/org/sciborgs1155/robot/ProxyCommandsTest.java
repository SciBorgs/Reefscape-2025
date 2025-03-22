package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.runToCompletion;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.lib.ProxyCommandGroups;

public class ProxyCommandsTest {
  private SubsystemBase subsystem;

  @BeforeEach
  void setup() {
    setupTests();
    subsystem = new SubsystemBase() {};
  }

  @AfterEach
  void destroy() throws Exception {
    reset();
  }

  @Test
  void sequenceTest() {
    Command command = ProxyCommandGroups.sequence(Commands.runOnce(() -> {}, subsystem));

    runToCompletion(command);

    assertTrue(command.getRequirements().size() == 0);
  }

  @Test
  void parallelTest() {
    Command command = ProxyCommandGroups.parallel(Commands.runOnce(() -> {}, subsystem));

    runToCompletion(command);

    assertTrue(command.getRequirements().size() == 0);
  }

  @Test
  void deadlineTest() {
    Command command =
        ProxyCommandGroups.deadline(
            Commands.runOnce(() -> {}, subsystem), Commands.run(() -> {}, subsystem));

    runToCompletion(command);

    assertTrue(command.getRequirements().size() == 0);
  }
}
