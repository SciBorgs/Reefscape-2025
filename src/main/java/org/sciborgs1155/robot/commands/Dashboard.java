package org.sciborgs1155.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private static NetworkTableEntry entryNewRequest;
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

    entryNewRequest = base.getEntry("newRequest");
    entryNewRequest.setBoolean(false);

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

  public static Trigger action() {
    return new Trigger(() -> entryNewRequest.getBoolean(false))
        .onTrue(Commands.run(() -> entryNewRequest.setBoolean(false)));
  }

  /** Returns the branch entry value (Branch A = "A"). Defaults to "". */
  public static String getBranchEntry() {
    return entryTargetBranch.getString("");
  }

  /** Returns the level entry value (L1 = 1, etc.). Defaults to -1. */
  public static int getLevelEntry() {
    return (int) entryTargetLevel.getInteger(-1);
  }

  /** Returns the algae entry value (Algae AB = 0, Algae CD = 1, etc.). Defaults to -1. */
  public static int getAlgaeEntry() {
    return (int) entryTargetAlgae.getInteger(-1);
  }
}
