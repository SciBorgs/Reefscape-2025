package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.Constants.Robot.BUMPER_LENGTH;
import static org.sciborgs1155.robot.Constants.alliance;
import static org.sciborgs1155.robot.Constants.allianceReflect;
import static org.sciborgs1155.robot.Constants.clamp;
import static org.sciborgs1155.robot.FieldConstants.Branch.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;

public class FieldConstants {
  // Origin at corner of blue alliance side of field

  public static final Distance LENGTH = Centimeters.of(1755);
  public static final Distance WIDTH = Centimeters.of(805);

  /** Returns whether the provided position is within the boundaries of the field. */
  public static boolean inField(Pose3d pose) {
    return (pose.getX() > 0
        && pose.getX() < LENGTH.in(Meters)
        && pose.getY() > 0
        && pose.getY() < WIDTH.in(Meters));
  }

  // TODO  hello!! IVAN!!! edit these as needed.

  // The minimum field-relative Y that the robot goes to. This dictates how close to the center
  // pillar the robot will try to score.
  public static final Distance BARGE_MIN_Y = WIDTH.div(2).plus(Feet.of(2));

  // The optimal X that the robot goes to. Increasing this value makes it go closer to the barge;
  // decreasing makes it go farther.
  public static final Distance BARGE_OPTIMAL_X = LENGTH.div(2).minus(Feet.of(4));

  /**
   * @param pose The field-relative pose of the robot.
   * @return The nearest position to go to for the barge.
   */
  public static Pose2d nearestBarge(Pose2d pose) {
    return allianceReflect( // I edited this comment muahahahah
        new Pose2d(
            BARGE_OPTIMAL_X.in(Meters),
            clamp(
                reflectDistance(pose.getMeasureY()).in(Meters),
                BARGE_MIN_Y.in(Meters),
                WIDTH.minus(BUMPER_LENGTH.div(2)).in(Meters)),
            Rotation2d.kZero));
  }

  /**
   * Reflects width-wise distances through the middle of the field if the alliance is red, otherwise
   * does nothing
   *
   * @param blueDist The input distance, usually for the blue alliance.
   * @return A reflected distance, only if the alliance is red.
   */
  private static Distance reflectDistance(Distance blueDist) {
    return alliance() == Alliance.Blue ? blueDist : WIDTH.minus(blueDist);
  }

  public static final Distance REEF_MIN_RADIUS = Centimeters.of(166 / 2);

  // The center of the blue alliance reef hexagon
  public static final Pose2d CENTER_REEF =
      new Pose2d(
          new Translation2d(Feet.of(12).plus(REEF_MIN_RADIUS), WIDTH.div(2)), Rotation2d.kZero);

  public static Alliance allianceFromPose(Pose2d pose) {
    return pose.getX() > LENGTH.in(Meters) / 2 ? Alliance.Red : Alliance.Blue;
  }

  // The field robot poses for blue alliance's reef branches A and B. Both are the farthest to the
  // blue
  // alliance side, and B is counter-clockwise of A.
  public static final Pose2d REEF_BRANCH_A =
      new Pose2d(
          CENTER_REEF
              .getMeasureX()
              .minus(REEF_MIN_RADIUS.plus(Constants.Robot.BUMPER_LENGTH.div(2))),
          CENTER_REEF.getMeasureY().plus(Inches.of(13 / 2)),
          Rotation2d.kZero);
  public static final Pose2d REEF_BRANCH_B =
      new Pose2d(
          CENTER_REEF
              .getMeasureX()
              .minus(REEF_MIN_RADIUS.plus(Constants.Robot.BUMPER_LENGTH.div(2))),
          CENTER_REEF.getMeasureY().minus(Inches.of(13 / 2)),
          Rotation2d.kZero);

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
          left.pose().getTranslation().plus(right.pose().getTranslation()).div(2),
          left.pose().getRotation());
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
      return Arrays.stream(Face.values())
          .filter(
              face -> face.pose().minus(pose.nearest(poseList())).getTranslation().getNorm() < 1e-4)
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

    private final Pose2d pose;

    private Branch(Pose2d pose) {
      this.pose = pose.transformBy(Constants.strafe(Inches.of(-3)));
    }

    public Pose2d pose() {
      return allianceReflect(pose);
    }

    /**
     * Moves the pose in or out depending on the level.
     *
     * @param level The level that the movement depends on.
     * @return A new pose moved to account for elevator tilt.
     */
    public Pose2d withLevel(Level level) {
      return level == Level.L4 ? pose().transformBy(Constants.advance(Inches.of(-4.5))) : pose();
    }

    /**
     * @return A list of all branch poses.
     */
    public static List<Pose2d> poseList() {
      return Arrays.stream(Branch.values()).map(b -> b.pose()).collect(Collectors.toList());
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
              .rotateAround(CENTER_REEF.getTranslation(), Rotation2d.fromDegrees(60).times(times)),
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
          .filter(branch -> branch.pose() == pose.nearest(poseList()))
          .findFirst()
          .orElse(A);
    }
  }

  public static enum Cage {
    LEFT(Pose2d.kZero),
    MID(Pose2d.kZero),
    RIGHT(Pose2d.kZero);

    private final Pose2d pose;

    Cage(Pose2d pose) {
      this.pose = pose;
    }

    public Pose2d pose() {
      return allianceReflect(pose);
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

  // private static final Distance SOURCE_MID_DISPLACEMENT = Inches.of(20);

  // Rotation of the top source (facing into the field)
  private static final Rotation2d SOURCE_ROTATION = Rotation2d.fromDegrees(90 - 144.011);

  private static final Pose2d LEFT_SOURCE =
      new Pose2d(
          new Translation2d(Units.inchesToMeters(33.526), Units.inchesToMeters(291.176))
              .plus(
                  new Translation2d(
                      Constants.Robot.BUMPER_LENGTH.in(Meters) / 2.0 + 0.05,
                      Rotation2d.fromRadians(SOURCE_ROTATION.getRadians()))),
          SOURCE_ROTATION);

  private static final Pose2d RIGHT_SOURCE =
      new Pose2d(
          new Translation2d(Units.inchesToMeters(33.526), Units.inchesToMeters(25.824))
              .plus(
                  new Translation2d(
                      Constants.Robot.BUMPER_LENGTH.in(Meters) / 2.0 + 0.05,
                      Rotation2d.fromRadians(SOURCE_ROTATION.unaryMinus().getRadians()))),
          SOURCE_ROTATION.unaryMinus());

  public static enum Source {
    LEFT_SOURCE_MID(LEFT_SOURCE),
    // LEFT_SOURCE_LEFT(LEFT_SOURCE.transformBy(strafe(SOURCE_MID_DISPLACEMENT.times(-1)))),
    // LEFT_SOURCE_RIGHT(LEFT_SOURCE.transformBy(strafe(SOURCE_MID_DISPLACEMENT))),
    RIGHT_SOURCE_MID(RIGHT_SOURCE);
    // RIGHT_SOURCE_LEFT(RIGHT_SOURCE.transformBy(strafe(SOURCE_MID_DISPLACEMENT.times(-1)))),
    // RIGHT_SOURCE_RIGHT(RIGHT_SOURCE.transformBy(strafe(SOURCE_MID_DISPLACEMENT)));

    private final Pose2d pose;

    Source(Pose2d pose) {
      this.pose = pose;
    }

    public Pose2d pose() {
      return allianceReflect(pose);
    }
  }

  // We don't have these poses. do NOT use them.
  // public static final Pose2d PROCESSOR = allianceReflect(Pose2d.kZero);

  // public static final Pose2d CAGE_1 = allianceReflect(Pose2d.kZero);
  // public static final Pose2d CAGE_2 = allianceReflect(Pose2d.kZero);
  // public static final Pose2d CAGE_3 = allianceReflect(Pose2d.kZero);

  // public static Pose2d nearestCage(Pose2d pose) {
  //   return pose.nearest(List.of(CAGE_1, CAGE_2, CAGE_3)).rotateBy(Constants.allianceRotation());
  // }
}
