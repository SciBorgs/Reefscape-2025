package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static java.util.Map.entry;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_EXTENSION;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MIN_EXTENSION;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import org.sciborgs1155.robot.Constants;
// import org.sciborgs1155.robot.Constants.Field.Branch;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;

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
  private static NetworkTableEntry entryTargetElevator;
  private static NetworkTableEntry entryRobotTick;
  private static NetworkTableEntry entryIsReal;
  private static NetworkTableEntry entryRequest;
  private static int tick;
  private static NetworkTableEntry entryBlueAlliance;
  private static NetworkTableEntry entryMatch;
  private static NetworkTableEntry entryMatchTime;
  public static final HashMap<String, NetworkTableEntry> info = new HashMap<>();
  public static Trigger processorTrigger;

  // private static final Branch[] branches = {
  //   Branch.A, Branch.B, Branch.C, Branch.D, Branch.E, Branch.F, Branch.G, Branch.H, Branch.I,
  //   Branch.J, Branch.K, Branch.L
  // };
  private static final Level[] levels = {Level.L1, Level.L2, Level.L3, Level.L4};

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

    entryTargetElevator = base.getEntry("elevator");
    entryTargetElevator.setDouble(MIN_EXTENSION.in(Meters));

    // Status
    entryRobotTick = base.getEntry("robotTick");
    entryRobotTick.setInteger(0);
    tick = 0;

    entryIsReal = base.getEntry("isReal");
    entryIsReal.setBoolean(Robot.isReal());

    entryRequest = base.getEntry("request");
    entryRequest.setString("");

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

  /** Returns a trigger for when a reef request from the Dashboard is recieved. */
  public static Trigger reef() {
    return new Trigger(
        () ->
            "reef".equals(entryRequest.getString(""))
                && (" ABCDEFGHIJKL".indexOf(entryTargetBranch.getString(" ")) - 1) != -1
                && 0 <= entryTargetLevel.getInteger(-1)
                && entryTargetLevel.getInteger(-1) <= 3);
  }

  /** Returns a trigger for when an algae request from the Dashboard is recieved. */
  public static Trigger algae() {
    return new Trigger(() -> "algae".equals(entryRequest.getString("")));
  }

  /** Returns a trigger for when a processor request from the Dashboard is recieved. */
  public static Trigger processor() {
    return new Trigger(() -> "processor".equals(entryRequest.getString("")));
  }

  /** Returns a trigger for when an elevator request from the Dashboard is recieved. */
  public static Trigger elevator() {
    return new Trigger(() -> "elevator".equals(entryRequest.getString("")));
  }

  // /** Returns the Branch that the branch entry is set to. Returns null if not found. */
  // public static Branch getBranchEntry() {
  //   int index = (" ABCDEFGHIJKL".indexOf(entryTargetBranch.getString(" ")) - 1);
  //   return index == -1 ? null : branches[index];
  // }

  /** Returns the branch entry value (Branch A = "A"). Defaults to "". */
  public static String getBranchEntry() {
    return entryTargetBranch.getString("");
  }

  /** Returns the Level that the level entry is set to. Returns null if not found. */
  public static Level getLevelEntry() {
    int index = (int) entryTargetLevel.getInteger(0) - 1;
    return index == -1 ? null : levels[index];
  }

  /** Returns the algae entry value (Algae AB = 0, Algae CD = 1, etc.). Defaults to -1. */
  public static int getAlgaeEntry() {
    return (int) entryTargetAlgae.getInteger(-1);
  }

  /** Returns the elevator height in meters. Defaults to elevator min extension. */
  public static double getElevatorEntry() {
    return entryTargetElevator.getDouble(MIN_EXTENSION.in(Meters))
            * (MAX_EXTENSION.in(Meters) - MIN_EXTENSION.in(Meters))
        + MIN_EXTENSION.in(Meters);
  }
}
