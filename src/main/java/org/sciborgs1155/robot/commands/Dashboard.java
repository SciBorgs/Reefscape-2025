package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_EXTENSION;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MIN_EXTENSION;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.FieldConstants.Branch;
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
  private static NetworkTableEntry entryCurrentElevator;
  private static NetworkTableEntry entryCameraFL;
  private static NetworkTableEntry entryCameraFR;
  private static NetworkTableEntry entryCameraBL;
  private static NetworkTableEntry entryCameraBR;
  private static NetworkTableEntry entryCameraBM;
  private static NetworkTableEntry entryInvertBeambreakSCL;
  private static NetworkTableEntry entryInvertBeambreakHPI;
  private static NetworkTableEntry entryRobotTick;
  private static NetworkTableEntry entryIsReal;
  private static NetworkTableEntry entryRequest;
  private static int tick;
  private static NetworkTableEntry entryBlueAlliance;
  private static NetworkTableEntry entryMatch;
  private static NetworkTableEntry entryMatchTime;

  private static final Branch[] branches = {
    Branch.A, Branch.B, Branch.C, Branch.D, Branch.E, Branch.F, Branch.G, Branch.H, Branch.I,
    Branch.J, Branch.K, Branch.L
  };
  private static final Level[] levels = {Level.L1, Level.L2, Level.L3, Level.L4};

  /** Sets up the dashboard. */
  public static boolean configure() {
    base = NetworkTableInstance.getDefault().getTable("Dashboard");

    // Scoring
    entryTargetBranch = base.getEntry("branch");
    entryTargetBranch.setString("");

    entryTargetLevel = base.getEntry("level");
    entryTargetLevel.setInteger(0);

    entryProcessor = base.getEntry("processor");
    entryProcessor.setBoolean(false);

    entryTargetAlgae = base.getEntry("algae");
    entryTargetAlgae.setInteger(-1);

    entryTargetElevator = base.getEntry("targetElevator");
    entryTargetElevator.setDouble(0);

    entryCurrentElevator = base.getEntry("currentElevator");
    entryCurrentElevator.setDouble(0);

    entryCameraFL = base.getEntry("cameraFL");
    entryCameraFL.getBoolean(true);

    entryCameraFR = base.getEntry("cameraFR");
    entryCameraFR.getBoolean(true);

    entryCameraBL = base.getEntry("cameraBL");
    entryCameraBL.getBoolean(true);

    entryCameraBR = base.getEntry("cameraBR");
    entryCameraBR.getBoolean(true);

    entryCameraBM = base.getEntry("cameraBM");
    entryCameraBM.getBoolean(true);

    entryInvertBeambreakSCL = base.getEntry("invertBeambreakSCL");
    entryInvertBeambreakSCL.getBoolean(false);

    entryInvertBeambreakHPI = base.getEntry("invertBeambreakHPI");
    entryInvertBeambreakHPI.getBoolean(false);

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
    return true;
  }

  /** Increments the robot tick value. Used by Dashboard to detect disconnects. */
  public static void tick() {
    tick += 1;
    entryRobotTick.setInteger(tick);
    entryMatchTime.setDouble(Robot.isReal() ? DriverStation.getMatchTime() : 0);
  }

  /** Returns a trigger for when a reef request from the Dashboard is recieved. */
  public static Trigger reef() {
    return new Trigger(() -> "reef".equals(entryRequest.getString("")));
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

  /** Returns the Branch that the branch entry is set to. Returns null if not found. */
  public static Branch getBranchEntry() {
    int index = (" ABCDEFGHIJKL".indexOf(entryTargetBranch.getString(" ")) - 1);
    return index == -1 ? null : branches[index];
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

  // * Returns a trigger based on the enabled status of camera front left. */
  public static Trigger cameraFL() {
    return new Trigger(() -> entryCameraFL.getBoolean(true));
  }

  // * Returns a trigger based on the enabled status of camera front right */
  public static Trigger cameraFR() {
    return new Trigger(() -> entryCameraFR.getBoolean(true));
  }

  // * Returns a trigger based on the enabled status of camera back left. */
  public static Trigger cameraBL() {
    return new Trigger(() -> entryCameraBL.getBoolean(true));
  }

  // * Returns a trigger based on the enabled status of camera back right. */
  public static Trigger cameraBR() {
    return new Trigger(() -> entryCameraBR.getBoolean(true));
  }

  // * Returns a trigger based on the enabled status of camera back middle. */
  public static Trigger cameraBM() {
    return new Trigger(() -> entryCameraBM.getBoolean(true));
  }

  // * Returns a whether the dashboard wants the scoral beambreak to be inverted.. */
  public static boolean invertBeambreakSCL() {
    return entryInvertBeambreakSCL.getBoolean(false);
  }

  // * Returns a whether the dashboard wants the human player intake beambreak to be inverted.. */
  public static boolean invertBeambreakHPI() {
    return entryInvertBeambreakHPI.getBoolean(false);
  }

  /**
   * Sends information about the current elevator height, as a percent.
   *
   * @param meters Current heigh in meters.
   */
  public static void setElevatorEntry(double meters) {
    entryCurrentElevator.setDouble(
        (meters - MIN_EXTENSION.in(Meters))
            / (MAX_EXTENSION.in(Meters) - MIN_EXTENSION.in(Meters)));
  }
}
