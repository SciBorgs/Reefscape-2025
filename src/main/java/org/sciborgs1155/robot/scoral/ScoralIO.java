package org.sciborgs1155.robot.scoral;

import monologue.Logged;

public interface ScoralIO extends Logged, AutoCloseable {

  void setPower(double power);

  boolean beambreak();
}
