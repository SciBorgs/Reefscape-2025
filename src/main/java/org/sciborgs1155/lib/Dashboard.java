package org.sciborgs1155.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Dashboard listens to NetworkTable information from the Reefscape-2025-Dashboard, which can be
 * used as Triggers.
 */
public class Dashboard {
  private final NetworkTable base;

  public Dashboard() {
    base = NetworkTableInstance.getDefault().getTable("Dashboard");
  }

  public void tick() {}
}
