package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.Field.Branch.*;
import static org.sciborgs1155.robot.Constants.Field.LENGTH;
import static org.sciborgs1155.robot.Constants.Field.WIDTH;
import static org.sciborgs1155.robot.Constants.Robot.BUMPER_LENGTH;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;

/**
 * Constants is a globally accessible class for storing immutable values. Every value should be
 * <code>public static final</code>.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * @see Units
 */
public class Constants {
  // TODO: Modify as needed.
  /** Returns the robot's alliance. */
  public static Alliance alliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  /** Returns the rotation of the robot's alliance with respect to the origin. */
  public static Rotation2d allianceRotation() {
    return Rotation2d.fromRotations(alliance() == Alliance.Blue ? 0 : 0.5);
  }

  public static enum RobotType {
    FULL,
    CHASSIS,
    NONE,
    COROLLING,
    SCORALING
  }

  /**
   * Returns the reflection of a pose about the center of the field, effectively swapping alliances.
   *
   * @param pose The pose being reflected.
   * @param allianceDependent If true, doesn't reflect if on the blue alliance. If false, reflects
   *     anyway.
   */
  public static Pose2d allianceReflect(Pose2d pose) {
    return alliance() == Alliance.Blue
        ? pose
        : new Pose2d(
            pose.getTranslation()
                .rotateAround(
                    new Translation2d(LENGTH.div(2), WIDTH.div(2)), Rotation2d.fromRotations(0.5)),
            pose.getRotation().plus(Rotation2d.fromRotations(0.5)));
  }

  public static RobotType ROBOT_TYPE = RobotType.COROLLING;

  /** Describes physical properites of the robot. */
  public static class Robot {
    public static final Mass MASS = Kilograms.of(25);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.2);

    public static final Distance SIDE_LENGTH = Inches.of(28);
    // TODO add bumper length to this

    public static final Distance BUMPER_LENGTH = Inches.of(28 + 3); // TODO NOT A FINAL VALUE
  }

  public static final Time PERIOD = Seconds.of(0.02); // roborio tickrate (s)
  public static final Time ODOMETRY_PERIOD = Seconds.of(1.0 / 250.0); // 4 ms (speedy!)
  public static final double DEADBAND = 0.15;
  public static final double MAX_RATE =
      DriveConstants.MAX_ACCEL.baseUnitMagnitude()
          / DriveConstants.MAX_ANGULAR_SPEED.baseUnitMagnitude();
  public static final double SLOW_SPEED_MULTIPLIER = 0.33;
  public static final double FULL_SPEED_MULTIPLIER = 1.0;
  public static final String CANIVORE_NAME = "drivetrain";

  // Origin at corner of blue alliance side of field
  public static class Field {
    public static final Distance LENGTH = Centimeters.of(1755);
    public static final Distance WIDTH = Centimeters.of(805);

    /** Returns whether the provided position is within the boundaries of the field. */
    public static boolean inField(Pose3d pose) {
      return (pose.getX() > 0
          && pose.getX() < Field.LENGTH.in(Meters)
          && pose.getY() > 0
          && pose.getY() < Field.WIDTH.in(Meters));
    }

    public static final Distance REEF_MIN_RADIUS = Centimeters.of(166 / 2);

    // The center of the blue alliance reef hexagon
    public static final Translation2d CENTER_REEF =
        new Translation2d(Feet.of(12).plus(REEF_MIN_RADIUS), WIDTH.div(2));

    // The field robot poses for blue alliance's reef branches A and B. Both are the farthest to the
    // blue
    // alliance side, and B is counter-clockwise of A.
    public static final Pose2d REEF_BRANCH_A =
        new Pose2d(
            CENTER_REEF.getMeasureX().minus(REEF_MIN_RADIUS.plus(BUMPER_LENGTH.div(2))),
            CENTER_REEF.getMeasureY().plus(Inches.of(13 / 2)),
            Rotation2d.fromRotations(0));
    public static final Pose2d REEF_BRANCH_B =
        new Pose2d(
            CENTER_REEF.getMeasureX().minus(REEF_MIN_RADIUS.plus(BUMPER_LENGTH.div(2))),
            CENTER_REEF.getMeasureY().minus(Inches.of(13 / 2)),
            Rotation2d.fromRotations(0));

    // Reef faces

    public static enum Face {
      AB(A, B),
      CD(C, D),
      EF(E, F),
      GH(G, H),
      IJ(I, J),
      KL(K, L);

      public final Branch left;
      public final Branch right;

      // Side (left or right)
      public static enum Side {
        LEFT,
        RIGHT
      }

      private Face(Branch left, Branch right) {
        this.left = left;
        this.right = right;
      }

      /**
       * @return The pose halfway in between the two branches. (The center of the reef face)
       */
      public Pose2d pose() {
        return new Pose2d(
            left.pose.getTranslation().plus(right.pose.getTranslation()).div(2),
            left.pose.getRotation());
      }

      /**
       * @return A list of the reef face center poses.
       */
      private static List<Pose2d> poseList() {
        return Arrays.stream(Face.values()).map(b -> b.pose()).collect(Collectors.toList());
      }

      public Branch branch(Side side) {
        return side == Side.LEFT ? left : right;
      }

      /**
       * Returns the nearest face to an input pose.
       *
       * @param pose The pose.
       * @return The nearest face to a pose.
       */
      public static Face nearest(Pose2d pose) {
        System.out.println(pose.nearest(Face.poseList()) + "\n");
        Arrays.stream(Face.values()).forEach(a -> System.out.println(a.pose().toString()));
        return Arrays.stream(Face.values())
            .filter(
                face ->
                    face.pose().minus(pose.nearest(poseList())).getTranslation().getNorm() < 1e-4)
            .findFirst()
            .orElse(AB);
      }
    }

    // Poses for scoraling.
    // A is the side of the reef closest to the barge, then B is clockwise of that, etc.
    // There are two reef branches per side, so the more counter-clockwise one is 1, and the
    // clockwise one is 2

    public static enum Branch {
      A(REEF_BRANCH_A),
      B(REEF_BRANCH_B),
      C(swapReefFace(REEF_BRANCH_A, 1)),
      D(swapReefFace(REEF_BRANCH_B, 1)),
      E(swapReefFace(REEF_BRANCH_A, 2)),
      F(swapReefFace(REEF_BRANCH_B, 2)),
      G(swapReefFace(REEF_BRANCH_A, 3)),
      H(swapReefFace(REEF_BRANCH_B, 3)),
      I(swapReefFace(REEF_BRANCH_A, 4)),
      J(swapReefFace(REEF_BRANCH_B, 4)),
      K(swapReefFace(REEF_BRANCH_A, 5)),
      L(swapReefFace(REEF_BRANCH_B, 5));

      public final Pose2d pose;

      private Branch(Pose2d pose) {
        this.pose = pose;
      }

      /**
       * @return The unit vector of the displacement from the center of the reef to the pose.
       */
      private Translation2d centerDisplacementUnit() {
        Translation2d diff = pose.getTranslation().minus(CENTER_REEF);
        return diff.div(diff.getNorm());
      }

      /**
       * Moves the pose in or out depending on the level.
       *
       * @param level The level that the movement depends on.
       * @return A new pose moved to account for elevator tilt.
       */
      public Pose2d withLevel(Level level) {
        return switch (level) {
          case L1, L2, L3 -> pose;
          case L4 ->
              new Pose2d(
                  pose.getTranslation().plus(centerDisplacementUnit().times(0.1)),
                  pose.getRotation());
          default -> pose; // hi ;P
        };
      }

      /**
       * @return The pose, moved slightly away from the reef face such that it is safe to retract
       *     the elevator.
       */
      public Pose2d backPose() {
        return new Pose2d(
            pose.getTranslation().plus(centerDisplacementUnit().times(0.5)), pose.getRotation());
      }

      /**
       * @return A list of all branch poses.
       */
      private static List<Pose2d> poseList() {
        return Arrays.stream(Branch.values()).map(b -> b.pose).collect(Collectors.toList());
      }

      /**
       * Moves the pose counter-clockwise to be relative to the next face of the reef hexagon.
       *
       * @param input The input pose.
       * @param times The number of sides, counter-clockwise to swap to.
       * @return A Pose2d that has been rotated counter-clockwise about the center of the reef.
       */
      private static Pose2d swapReefFace(Pose2d input, int times) {
        return new Pose2d(
            input
                .getTranslation()
                .rotateAround(CENTER_REEF, Rotation2d.fromDegrees(60).times(times)),
            Rotation2d.fromDegrees(60 * times));
      }

      /**
       * Returns the nearest branch to an input pose.
       *
       * @param pose The pose.
       * @return The nearest branch to a pose.
       */
      public static Branch nearest(Pose2d pose) {
        return Arrays.stream(Branch.values())
            .filter(branch -> branch.pose == pose.nearest(poseList()))
            .findFirst()
            .orElse(A);
      }
    }

    public static enum Cage {
      LEFT(new Pose2d()),
      MID(new Pose2d()),
      RIGHT(new Pose2d());

      public final Pose2d pose;

      Cage(Pose2d pose) {
        this.pose = pose;
      }

      /**
       * Returns the nearest cage to an input pose.
       *
       * @param pose The pose.
       * @return The nearest cage to the pose.
       */
      public static Cage nearest(Pose2d pose) {
        return Arrays.stream(Cage.values())
            .filter(
                (cage) ->
                    cage.pose
                        == pose.nearest(
                            Arrays.stream(Cage.values())
                                .map(b -> b.pose)
                                .collect(Collectors.toList())))
            .findFirst()
            .orElse(LEFT);
      }
    }

    public static enum Source {
      LEFT(
          new Pose2d(
              new Translation2d(Units.inchesToMeters(33.526), Units.inchesToMeters(291.176))
                  .plus(
                      new Translation2d(
                          BUMPER_LENGTH.div(2).in(Meters) + 0.05,
                          Rotation2d.fromRadians(SOURCE_ROTATION.getRadians()))),
              SOURCE_ROTATION.rotateBy(Rotation2d.k180deg))),
      RIGHT(
          new Pose2d(
              new Translation2d(Units.inchesToMeters(33.526), Units.inchesToMeters(25.824))
                  .plus(
                      new Translation2d(
                          BUMPER_LENGTH.div(2).in(Meters) + 0.05,
                          Rotation2d.fromRadians(SOURCE_ROTATION.unaryMinus().getRadians()))),
              SOURCE_ROTATION.unaryMinus().rotateBy(Rotation2d.k180deg)));

      public Pose2d pose;

      Source(Pose2d pose) {
        this.pose = pose;
      }
    }

    // Rotation of the top source (facing into the field)
    private static final Rotation2d SOURCE_ROTATION = Rotation2d.fromDegrees(90 - 144.011);

    public static final Pose2d PROCESSOR = allianceReflect(new Pose2d());

    public static final Pose2d CAGE_1 = allianceReflect(new Pose2d());
    public static final Pose2d CAGE_2 = allianceReflect(new Pose2d());
    public static final Pose2d CAGE_3 = allianceReflect(new Pose2d());

    public static Pose2d nearestCage(Pose2d pose) {
      return pose.nearest(List.of(CAGE_1, CAGE_2, CAGE_3)).rotateBy(allianceRotation());
    }
  }
}
