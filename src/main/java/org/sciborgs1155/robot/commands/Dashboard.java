package org.sciborgs1155.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

/**
 * Dashboard listens to NetworkTable information from the Reefscape-2025-Dashboard, which can be
 * used as Triggers. Must call the configure() method.
 */
public class Dashboard {
  private static NetworkTable base;
  private static NetworkTableEntry entryTargetBranch;
  private static NetworkTableEntry entryTargetLevel;
  private static NetworkTableEntry entryProcessor;
  private static NetworkTableEntry entryTargetAlgae;
  private static NetworkTableEntry entryRobotTick;
  private static int tick;
  private static NetworkTableEntry entryBlueAlliance;
  private static NetworkTableEntry entryMatch;
  private static NetworkTableEntry entryMatchTime;
  public static final HashMap<String, NetworkTableEntry> info = new HashMap<>();
  public static Trigger processorTrigger;

  /** Sets up the dashboard. */
  public static void configure() {
    base = NetworkTableInstance.getDefault().getTable("Dashboard");

    // Scoring
    entryTargetBranch = base.getEntry("branch");
    entryTargetBranch.setString("");

    entryTargetLevel = base.getEntry("level");
    entryTargetLevel.setInteger(0);

    entryProcessor = base.getEntry("processor");
    entryProcessor.setBoolean(false);
    processorTrigger = new Trigger(() -> entryProcessor.getBoolean(false));

    entryTargetAlgae = base.getEntry("algae");
    entryTargetAlgae.setInteger(-1);

    // Status
    entryRobotTick = base.getEntry("robotTick");
    entryRobotTick.setInteger(0);
    tick = 0;

    // Match
    entryBlueAlliance = base.getEntry("blueAlliance");
    entryBlueAlliance.setBoolean(Constants.alliance() == Alliance.Blue);

    entryMatch = base.getEntry("match");
    entryMatchTime = base.getEntry("matchTime");
    if (Robot.isReal()) {

      String temp = "";
      switch (DriverStation.getMatchType()) {
        case None:
          temp = "M";
        case Elimination:
          temp = "E";
        case Practice:
          temp = "P";
        case Qualification:
          temp = "Q";
        default:
          temp = "M";
      }
      entryMatch.setString(
          "@ " + DriverStation.getEventName() + " / " + temp + DriverStation.getMatchNumber());
      entryMatchTime.setDouble(DriverStation.getMatchTime());
    } else {
      entryMatch.setString("@ Sim / M0");
      entryMatchTime.setDouble(0);
    }

    // info setup
    transmit("closestBranch");
  }

  /** Increments the robot tick value. Used by Dashboard to detect disconnects. */
  public static void tick() {
    tick += 1;
    entryRobotTick.setInteger(tick);
    entryMatchTime.setDouble(Robot.isReal() ? DriverStation.getMatchTime() : 0);
  }

  /**
   * Adds a key/entry pair to the info hashmap.
   *
   * @param key the key of the NetworkTables entry
   */
  public static void transmit(String key) {
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
    return new Trigger(() -> (level == entryTargetLevel.getInteger(-1)));
  }

  /**
   * Returns a Trigger, given a reef algae.
   *
   * @param side the side of the reef, with AB as 0
   * @return a Trigger for that algae
   */
  private static Trigger setTriggerAlgae(int side) {
    return new Trigger(() -> (side == entryTargetAlgae.getInteger(-1)));
  }

  /** An enum for each branch of the alliance's reef. */
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

  /** An enum for each level of a branch of the alliance's reef. */
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

  /** An enum for each algae of the alliance's reef. */
  public static enum Algae {
    AB(0),
    CD(1),
    EF(2),
    GH(3),
    IJ(4),
    KL(5);

    public final int algae;
    public final Trigger trigger;

    private Algae(int algae) {
      this.algae = algae;
      this.trigger = setTriggerAlgae(algae);
    }
  }
}
