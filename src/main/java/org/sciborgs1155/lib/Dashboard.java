package org.sciborgs1155.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Dashboard listens to NetworkTable information from the Reefscape-2025-Dashboard, which can be
 * used as Triggers.
 */
public class Dashboard {
  private final NetworkTable base;
  private final NetworkTableEntry entryTargetBranch;
  private final NetworkTableEntry entryTargetLevel;

  public Dashboard() {
    base = NetworkTableInstance.getDefault().getTable("Dashboard");
    entryTargetBranch = base.getEntry("branch");
    entryTargetLevel = base.getEntry("level");
  }

  private Trigger setTriggerBranch(String branch) {
    return new Trigger(() -> (branch == entryTargetBranch.getString("")));
  }

  private Trigger setTriggerLevel(int level) {
    return new Trigger(() -> (level == entryTargetLevel.getInteger(0)));
  }

  public Trigger BA() {
    return setTriggerBranch("A");
  }

  public Trigger BB() {
    return setTriggerBranch("B");
  }

  public Trigger BC() {
    return setTriggerBranch("C");
  }

  public Trigger BD() {
    return setTriggerBranch("D");
  }

  public Trigger BE() {
    return setTriggerBranch("E");
  }

  public Trigger BF() {
    return setTriggerBranch("F");
  }

  public Trigger BG() {
    return setTriggerBranch("G");
  }

  public Trigger BH() {
    return setTriggerBranch("H");
  }

  public Trigger BI() {
    return setTriggerBranch("I");
  }

  public Trigger BJ() {
    return setTriggerBranch("J");
  }

  public Trigger BK() {
    return setTriggerBranch("K");
  }

  public Trigger BL() {
    return setTriggerBranch("L");
  }

  public Trigger L1() {
    return setTriggerLevel(1);
  }

  public Trigger L2() {
    return setTriggerLevel(2);
  }

  public Trigger L3() {
    return setTriggerLevel(3);
  }

  public Trigger L4() {
    return setTriggerLevel(4);
  }
}
