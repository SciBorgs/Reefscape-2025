package org.sciborgs1155.robot;

import java.util.HashMap;
import java.util.Map;

public final class Ports {
  // TODO: Add and change all ports as needed.

  public static Map<Integer, String> idToName = new HashMap<>();

  private static int name(int port, String nickname) {
    idToName.put(port, nickname);
    System.out.println(port);
    return port;
  }

  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  public static final class Drive {
    public static final int CANANDGYRO = 50;
    public static final int FRONT_LEFT_DRIVE = name(9, "FL drive");
    public static final int REAR_LEFT_DRIVE = name(12, "RL drive");
    public static final int FRONT_RIGHT_DRIVE = name(11, "FR drive");
    public static final int REAR_RIGHT_DRIVE = name(10, "RR drive");

    public static final int FRONT_LEFT_TURNING = name(4, "FL turn");
    public static final int REAR_LEFT_TURNING = name(1, "RL turn");
    public static final int FRONT_RIGHT_TURNING = name(3, "FR turn");
    public static final int REAR_RIGHT_TURNING = name(2, "RR turn");

    public static final int FRONT_LEFT_CANCODER = name(5, "FL cancoder");
    public static final int REAR_LEFT_CANCODER = name(7, "RL cancoder");
    public static final int FRONT_RIGHT_CANCODER = name(6, "FR cancoder");
    public static final int REAR_RIGHT_CANCODER = name(8, "RR cancoder");
  }

  public static final class GroundIntake {
    public static final int ARM_MOTOR = name(18, "ground arm");
    public static final int ROLLER_MOTOR = name(19, "ground roller");
    public static final int CANCODER = name(20, "ground cancoder"); // TODO change i think
  }

  public static final class Elevator {
    public static final int FRONT_LEADER =
        name(14, "front elevator"); // TODO update with actual position
    public static final int BACK_FOLLOWER = name(13, "back elevator");
  }

  public static final class Scoral {
    public static final int ROLLER = name(15, "scoral roller");
    public static final int BEAMBREAK = name(23, "scoral beambreak");
  }

  public static final class LEDs {
    public static final int LED_PORT = 9;
  }

  public static final class Hopper {
    public static final int MOTOR = name(-1, "hopper");
    public static final int BEAMBREAK = name(-2, "hopper beambreak");
  }
}
