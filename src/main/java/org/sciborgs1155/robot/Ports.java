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

  public static final class GroundIntake {
    public static final int ARM_MOTOR = 18;
    public static final int ROLLER_MOTOR = 19;
    public static final int CANCODER = 2; // TODO change i think
  }

  public static final class Elevator {
    public static final int LEADER = 18;
    public static final int FOLLOWER = 19;
  }

  public static final class Scoral {
    public static final int ROLLER = 21;
    public static final int BEAMBREAK = 22;
  }

  public static final class LEDs {
    public static final int LED_PORT = 2;
  }

  public static final class Hopper {
    public static final int MOTOR = -1;
    public static final int BEAMBREAK = -1;
  }
}
