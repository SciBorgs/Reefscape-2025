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
    entryTargetBranch.setString("");
    entryTargetLevel = base.getEntry("level");
    entryTargetLevel.setInteger(0);
  }

  private Trigger setTriggerBranch(String branch) {
    return new Trigger(() -> (branch == entryTargetBranch.getString("")));
  }

  private Trigger setTriggerLevel(int level) {
    return new Trigger(() -> (level == entryTargetLevel.getInteger(0)));
  }

  public Trigger A() {
    return setTriggerBranch("A");
  }

  public Trigger B() {
    return setTriggerBranch("B");
  }

  public Trigger C() {
    return setTriggerBranch("C");
  }

  public Trigger D() {
    return setTriggerBranch("D");
  }

  public Trigger E() {
    return setTriggerBranch("E");
  }

  public Trigger F() {
    return setTriggerBranch("F");
  }

  public Trigger G() {
    return setTriggerBranch("G");
  }

  public Trigger H() {
    return setTriggerBranch("H");
  }

  public Trigger I() {
    return setTriggerBranch("I");
  }

  public Trigger J() {
    return setTriggerBranch("J");
  }

  public Trigger K() {
    return setTriggerBranch("K");
  }

  public Trigger L() {
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
