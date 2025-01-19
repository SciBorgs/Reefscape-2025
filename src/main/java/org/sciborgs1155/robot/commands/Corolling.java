package org.sciborgs1155.robot.commands;

import org.sciborgs1155.robot.arm.Arm;
import org.sciborgs1155.robot.coroller.Coroller;

public class Corolling {
  private Arm arm;
  private Coroller coroller;

  public Corolling(Arm arm, Coroller coroller) {
    this.arm = arm;
    this.coroller = coroller;
  }
}
