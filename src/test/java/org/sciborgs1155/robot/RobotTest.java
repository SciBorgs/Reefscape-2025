package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.robot.drive.DriveConstants.driveSimAdded;

import org.junit.jupiter.api.Test;

public class RobotTest {
  @Test
  void setup() throws Exception {
    new Robot().close();
    driveSimAdded = true;
    reset();
  }
}
