package org.sciborgs1155.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

import org.sciborgs1155.robot.Constants;

/**
 * Dashboard listens to NetworkTable information from the Reefscape-2025-Dashboard, which can be
 * used as Triggers. Must call the configure() method
 */
public class Dashboard {
  private static NetworkTable base;
  private static NetworkTableEntry entryTargetBranch;
  private static NetworkTableEntry entryTargetLevel;
  private static NetworkTableEntry entryRobotConnected;
  private static NetworkTableEntry entryBlueAlliance;
  public final static HashMap<String, NetworkTableEntry> info = new HashMap<>();
  
  /** Setups up dashboard */
  public static void configure() {
    base = NetworkTableInstance.getDefault().getTable("Dashboard");
    entryTargetBranch = base.getEntry("branch");
    entryTargetBranch.setString("");
    entryTargetLevel = base.getEntry("level");
    entryTargetLevel.setInteger(0);
    entryRobotConnected = base.getEntry("robotConnected");
    entryRobotConnected.setBoolean(true);
    entryBlueAlliance = base.getEntry("blueAlliance");
    entryBlueAlliance.setBoolean(Constants.alliance() == Alliance.Blue);

    //info setup
    upload("closestBranch");
  }

  /**
   * adds the key/entry pair to the info hashmap
   */
  public static void upload(String key) {
    NetworkTableEntry entry = base.getEntry(key);
    info.put(key, entry);
  }

  /**
   * Returns a Trigger, given a reef branch.
   *
   * @param branch the branch for the trigger (A-L)
   * @return a Trigger for that branch
   */
  private static Trigger setTriggerBranch(String branch) {
    return new Trigger(() -> (branch.equals(entryTargetBranch.getString(""))));
  }

  /**
   * Returns a Trigger, given a reef branch level.
   *
   * @param level the level for the trigger (1-4)
   * @return a Trigger for that level
   */
  private static Trigger setTriggerLevel(int level) {
    return new Trigger(() -> (level == entryTargetLevel.getInteger(0)));
  }

  public static enum Branches {
    A("A"),
    B("B"),
    C("C"),
    D("D"),
    E("E"),
    F("F"),
    G("G"),
    H("H"),
    I("I"),
    J("J"),
    K("K"),
    L("L");

    public final String branch;
    public final Trigger trigger;

    private Branches(String branch) {
      this.branch = branch;
      this.trigger = setTriggerBranch(branch);
    }
  }

  public static enum Levels {
    L1(1),
    L2(2),
    L3(3),
    L4(4);

    public final int level;
    public final Trigger trigger;

    private Levels(int level) {
      this.level = level;
      this.trigger = setTriggerLevel(level);
    }
  }
}
