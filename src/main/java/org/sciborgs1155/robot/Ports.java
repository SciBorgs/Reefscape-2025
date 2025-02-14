package org.sciborgs1155.robot;

public final class Ports {
  // TODO: Add and change all ports as needed.
  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  public static final class Drive {
    public static final int CANANDGYRO = 50;
    public static final int FRONT_LEFT_DRIVE = 9;
    public static final int REAR_LEFT_DRIVE = 12;
    public static final int FRONT_RIGHT_DRIVE = 11;
    public static final int REAR_RIGHT_DRIVE = 10;

    public static final int FRONT_LEFT_TURNING = 4;
    public static final int REAR_LEFT_TURNING = 1;
    public static final int FRONT_RIGHT_TURNING = 3;
    public static final int REAR_RIGHT_TURNING = 2;

    public static final int FRONT_LEFT_CANCODER = 5;
    public static final int REAR_LEFT_CANCODER = 7;
    public static final int FRONT_RIGHT_CANCODER = 6;
    public static final int REAR_RIGHT_CANCODER = 8;
  }

  public static final class GroundIntake {
    public static final int ARM_MOTOR = 18;
    public static final int ROLLER_MOTOR = 19;
    public static final int CANCODER = 20; // TODO change i think
  }

  public static final class Elevator {
    public static final int FRONT_LEADER = 14;
    public static final int BACK_FOLLOWER = 13;
  }

  public static final class Scoral {
    public static final int ROLLER = 15;
    public static final int BEAMBREAK = 23;
  }

  public static final class LEDs {
    public static final int LED_PORT = 9;
  }

  public static final class Hopper {
    public static final int MOTOR = -1;
    public static final int BEAMBREAK = -1;
  }
}
