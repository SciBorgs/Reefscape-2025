package org.sciborgs1155.robot;

public final class Ports {
  // TODO: Add and change all ports as needed.
  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  public static final class Drive {
    public static final int CANANDGYRO = 20;
    public static final int FRONT_LEFT_DRIVE = 11;
    public static final int REAR_LEFT_DRIVE = 10;
    public static final int FRONT_RIGHT_DRIVE = 12;
    public static final int REAR_RIGHT_DRIVE = 13;

    public static final int FRONT_LEFT_TURNING = 15;
    public static final int REAR_LEFT_TURNING = 14;
    public static final int FRONT_RIGHT_TURNING = 16;
    public static final int REAR_RIGHT_TURNING = 17;
  }

  public static final class Elevator {
    public static final int LEADER = -1;
    public static final int FOLLOWER = -1;
  }

  public static final class Scoral {
    public static final int TOP_ROLLER = 21;
    public static final int BOTTOM_ROLLER = 22;
    public static final int BEAMBREAK = 23;
  }

  public static final class Hopper {
    public static final int LEFT_MOTOR = -1;
    public static final int RIGHT_MOTOR = -1;
    public static final int BEAMBREAK = -1;
  }
}
