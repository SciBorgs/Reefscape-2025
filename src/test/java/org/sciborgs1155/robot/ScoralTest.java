package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;

import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.scoral.Scoral;

public class ScoralTest {
  Scoral scoral;

  @Test
  public void init() throws Exception {
    scoral = Scoral.create();
    runUnitTest(scoral.outtakeTest());
    scoral.close();
    reset();
  }
}
