package org.sciborgs1155.robot.commands.Dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import org.sciborgs1155.robot.Constants;

/**
 * Dashboard listens to NetworkTable information from the Reefscape-2025-Dashboard, which can be
 * used as Triggers. Must call the configure() method.
 */
public class Dashboard {
  private final Display display = new Display();
  private String selectedBranch = "";
  private int selectedLevels = 0;

  /** Sets up the dashboard. */
  public Dashboard() {
  }

  /**
   * Returns a Trigger, given a reef branch.
   *
   * @param branch the branch for the trigger (A-L)
   * @return a Trigger for that branch
   */
  private Trigger setTriggerBranch(String branch) {
    return new Trigger(() -> (branch.equals(selectedBranch)));
  }

  /**
   * Returns a Trigger, given a reef branch level.
   *
   * @param level the level for the trigger (1-4)
   * @return a Trigger for that level
   */
  private Trigger setTriggerLevel(int level) {
    return new Trigger(() -> (level == selectedLevels));
  }

  /** An enum for each branch of the alliance's reef. */
  public enum Branches {
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
  public enum Levels {
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
