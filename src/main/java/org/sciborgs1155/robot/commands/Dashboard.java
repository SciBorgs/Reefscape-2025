package org.sciborgs1155.robot.commands;

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
  private final NetworkTableEntry entryRobotConnected;

  /** Creates a Dashboard object. */
  public Dashboard() {
    // setup NetworkTables
    base = NetworkTableInstance.getDefault().getTable("Dashboard");
    entryTargetBranch = base.getEntry("branch");
    entryTargetBranch.setString("");
    entryTargetLevel = base.getEntry("level");
    entryTargetLevel.setInteger(0);
    entryRobotConnected = base.getEntry("robotConnected");
    entryRobotConnected.setBoolean(true);
  }

  /**
   * Returns a Trigger, given a reef branch.
   *
   * @param branch the branch for the trigger (A-L)
   * @return a Trigger for that branch
   */
  private Trigger setTriggerBranch(String branch) {
    return new Trigger(() -> (branch.equals(entryTargetBranch.getString(""))));
  }

  /**
   * Returns a Trigger, given a reef branch level.
   *
   * @param level the level for the trigger (1-4)
   * @return a Trigger for that level
   */
  private Trigger setTriggerLevel(int level) {
    return new Trigger(() -> (level == entryTargetLevel.getInteger(0)));
  }

  public Trigger a() {
    return setTriggerBranch("A");
  }

  public Trigger b() {
    return setTriggerBranch("B");
  }

  public Trigger c() {
    return setTriggerBranch("C");
  }

  public Trigger d() {
    return setTriggerBranch("D");
  }

  public Trigger e() {
    return setTriggerBranch("E");
  }

  public Trigger f() {
    return setTriggerBranch("F");
  }

  public Trigger g() {
    return setTriggerBranch("G");
  }

  public Trigger h() {
    return setTriggerBranch("H");
  }

  public Trigger i() {
    return setTriggerBranch("I");
  }

  public Trigger j() {
    return setTriggerBranch("J");
  }

  public Trigger k() {
    return setTriggerBranch("K");
  }

  public Trigger l() {
    return setTriggerBranch("L");
  }

  public Trigger level1() {
    return setTriggerLevel(1);
  }

  public Trigger level2() {
    return setTriggerLevel(2);
  }

  public Trigger level3() {
    return setTriggerLevel(3);
  }

  public Trigger level4() {
    return setTriggerLevel(4);
  }
}
