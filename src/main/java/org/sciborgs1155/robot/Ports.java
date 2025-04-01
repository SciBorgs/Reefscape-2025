package org.sciborgs1155.robot;

import static java.util.Map.entry;

import java.util.Map;

public final class Ports {
  public static final Map<Integer, String> idToName =
      Map.ofEntries(
          entry(Drive.FRONT_LEFT_DRIVE, "FL drive"),
          entry(Drive.REAR_LEFT_DRIVE, "RL drive"),
          entry(Drive.FRONT_RIGHT_DRIVE, "FR drive"),
          entry(Drive.REAR_RIGHT_DRIVE, "RR drive"),
          entry(Drive.FRONT_LEFT_TURNING, "FL turn"),
          entry(Drive.REAR_LEFT_TURNING, "RL turn"),
          entry(Drive.FRONT_RIGHT_TURNING, "FR turn"),
          entry(Drive.REAR_RIGHT_TURNING, "RR turn"),
          entry(Drive.FRONT_LEFT_CANCODER, "FL cancoder"),
          entry(Drive.REAR_LEFT_CANCODER, "RL cancoder"),
          entry(Drive.FRONT_RIGHT_CANCODER, "FR cancoder"),
          entry(Drive.REAR_RIGHT_CANCODER, "RR cancoder"),
          entry(GroundIntake.ARM_LEADER, "ground left arm leader"),
          entry(GroundIntake.ARM_FOLLOWER, "ground right arm follower"),
          entry(GroundIntake.ROLLER_MOTOR, "ground roller"),
          entry(GroundIntake.CANCODER, "ground intake"),
          entry(Elevator.FRONT_LEADER, "front elevator"),
          entry(Elevator.BACK_FOLLOWER, "back elevator"),
          entry(Scoral.SCORAL, "scoral roller"),
          entry(Scoral.ALGAE, "algae roller"),
          entry(Hopper.MOTOR, "hopper"));

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
    public static final int ARM_LEADER = 99;
    public static final int ARM_FOLLOWER = 101;
    public static final int ROLLER_MOTOR = 102;
    public static final int CANCODER = 103;
  }

  public static final class Elevator {
    public static final int FRONT_LEADER = 14;
    public static final int BACK_FOLLOWER = 13;
  }

  public static final class Scoral {
    public static final int SCORAL = 15;
    public static final int ALGAE = 17;
    public static final int BEAMBREAK = 1;
  }

  public static final class LEDs {
    public static final int LED_PORT = 9;
  }

  public static final class Hopper {
    public static final int MOTOR = 18;
    public static final int BEAMBREAK = 3;
  }
}
