package org.sciborgs1155.robot.scoral;

import monologue.Logged;

public interface ScoralIO extends Logged, AutoCloseable {

  /** Sets the motor power. */
  void setPower(double power);

  /** Returns the value of the beambreak. */
  boolean beambreak();

  /** Returns the angular velocity of the motor, in radians per second. */
  double getAngularVelocity();
}
