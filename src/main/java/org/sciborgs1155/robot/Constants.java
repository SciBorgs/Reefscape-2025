package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.Field.LENGTH;
import static org.sciborgs1155.robot.Constants.Field.WIDTH;

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
import java.util.List;
import java.util.stream.Collectors;
import org.sciborgs1155.robot.drive.DriveConstants;

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

  /**
   * Returns the reflection of a pose about the center of the field, effectively swapping alliances.
   *
   * @param pose The pose being reflected.
   * @param allianceDependent If true, doesn't reflect if on the blue alliance. If false, reflects
   *     anyway.
   */
  public static Pose2d allianceReflect(Pose2d pose, boolean allianceDependent) {
    return alliance() == Alliance.Blue && allianceDependent
        ? pose
        : new Pose2d(
            pose.getTranslation()
                .rotateAround(
                    new Translation2d(LENGTH.div(2), WIDTH.div(2)), Rotation2d.fromRotations(0.5)),
            pose.getRotation().plus(Rotation2d.fromRotations(0.5)));
  }

  /**
   * Returns the reflection of a pose about the center point of the field, only if the alliance is
   * not blue.
   *
   * @param pose The pose being reflected.
   */
  public static Pose2d allianceReflect(Pose2d pose) {
    return allianceReflect(pose, true);
  }

  /** Describes physical properites of the robot. */
  public static class Robot {
    public static final Mass MASS = Kilograms.of(25);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.2);
  }

  public static final Time PERIOD = Seconds.of(0.02); // roborio tickrate (s)
  public static final double DEADBAND = 0.15;
  public static final double MAX_RATE =
      DriveConstants.MAX_ACCEL.baseUnitMagnitude()
          / DriveConstants.MAX_ANGULAR_SPEED.baseUnitMagnitude();
  public static final double SLOW_SPEED_MULTIPLIER = 0.33;
  public static final double FULL_SPEED_MULTIPLIER = 1.0;

  // Origin at corner of blue alliance side of field
  public static class Field {
    public static final Distance LENGTH = Centimeters.of(1775);
    public static final Distance WIDTH = Centimeters.of(805);

    /** Returns whether the provided position is within the boundaries of the field. */
    public static boolean inField(Pose3d pose) {
      return (pose.getX() > 0
          && pose.getX() < Field.LENGTH.in(Meters)
          && pose.getY() > 0
          && pose.getY() < Field.WIDTH.in(Meters));
    }

    public enum Level {
      L1(.3),
      L2(.7),
      L3(1),
      L4(1.5);

      public final double height;

      Level(double height) {
        this.height = height;
      }

      public double getHeight() {
        return this.height;
      }
    }

    // The field poses for blue alliance's reef branches A and B. Both are the farthest to the blue
    // alliance side, and B is counter-clockwise of A.
    public static final Pose2d REEF_BRANCH_A = new Pose2d();
    public static final Pose2d REEF_BRANCH_B = new Pose2d();

    // Poses for scoraling.
    // A is the side of the reef closest to the barge, then B is clockwise of that, etc.
    // There are two reef branches per side, so the more counter-clockwise one is 1, and the
    // clockwise one is 2

    public static enum Branch {
      A(REEF_BRANCH_A),
      B(REEF_BRANCH_B),
      C(swapSides(REEF_BRANCH_A, 1)),
      D(swapSides(REEF_BRANCH_B, 1)),
      E(swapSides(REEF_BRANCH_A, 2)),
      F(swapSides(REEF_BRANCH_B, 2)),
      G(swapSides(REEF_BRANCH_A, 3)),
      H(swapSides(REEF_BRANCH_B, 3)),
      I(swapSides(REEF_BRANCH_A, 4)),
      J(swapSides(REEF_BRANCH_B, 4)),
      K(swapSides(REEF_BRANCH_A, 5)),
      L(swapSides(REEF_BRANCH_B, 5));

      public final Pose2d pose;

      Branch(Pose2d pose) {
        this.pose = pose;
      }

      public static List<Pose2d> poseList() {
        return List.of(Branch.values()).stream().map(b -> b.pose).collect(Collectors.toList());
      }

      /**
       * Moves the pose counter-clockwise to the next side of the
       *
       * @param input
       * @param times
       * @return
       */
      private static Pose2d swapSides(Pose2d input, int times) {
        return new Pose2d(
            input
                .getTranslation()
                .rotateAround(CENTER_REEF, Rotation2d.fromDegrees(-60).times(times)),
            Rotation2d.fromDegrees(60).times(times));
      }

      public static Branch nearest(Pose2d pose) {
        for (int i = 0; i < values().length; i++) {
          if (Branch.values()[i].pose == pose.nearest(poseList())) {
            return Branch.values()[i];
          }
        }
        return L;
      }
    }

    // The center of the reef hexagon
    public static final Translation2d CENTER_REEF =
        new Translation2d(Inches.of(65.5 / 2).plus(Feet.of(12)), WIDTH.div(2));

    public static enum Cage {
      LEFT,
      MID,
      RIGHT;
    }

    // Not poses of the game element itself, rather the needed pose of the robot to use it.
    public static final Pose2d LEFT_SOURCE = allianceReflect(new Pose2d());
    public static final Pose2d RIGHT_SOURCE = allianceReflect(new Pose2d());

    public static final Pose2d PROCESSOR = allianceReflect(new Pose2d());

    public static final Pose2d CAGE_1 = allianceReflect(new Pose2d());
    public static final Pose2d CAGE_2 = allianceReflect(new Pose2d());
    public static final Pose2d CAGE_3 = allianceReflect(new Pose2d());

    public static Pose2d nearestCage(Pose2d pose) {
      return pose.nearest(List.of(CAGE_1, CAGE_2, CAGE_3)).rotateBy(allianceRotation());
    }
  }
}
