package org.sciborgs1155.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.sciborgs1155.robot.vision.Vision.CameraConfig;

public class VisionConstants {
  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // WARNING: EMPTY TRANSFORMS WILL CRASH CODE UPON TAG DETECTION
  public static final CameraConfig BACK_LEFT_CAMERA =
      new CameraConfig(
          "back left",
          new Transform3d(
              Inches.of(-12).in(Meters),
              Inches.of(13).in(Meters),
              Inches.of(9.375).in(Meters),
              //   new Rotation3d(0, Math.toRadians(-25), Math.toRadians(108.25))));
              new Rotation3d(Radians.zero(), Degrees.of(-25), Radians.zero())
                  .rotateBy(new Rotation3d(Radians.zero(), Radians.zero(), Degrees.of(108.25)))));

  public static final CameraConfig BACK_RIGHT_CAMERA =
      new CameraConfig(
          "back right",
          new Transform3d(
              Inches.of(-12).in(Meters),
              Inches.of(-13).in(Meters),
              Inches.of(9.375).in(Meters),
              //   new Rotation3d(0, Math.toRadians(-25), Math.toRadians(-108.25))));
              new Rotation3d(Radians.zero(), Degrees.of(-25), Degrees.of(0))
                  .rotateBy(new Rotation3d(Radians.zero(), Degrees.of(0), Degrees.of(-108.25)))));
  public static final CameraConfig BACK_MIDDLE_CAMERA =
      new CameraConfig(
          "back middle",
          new Transform3d(
              Inches.of(-8.358).in(Meters),
              Inches.of(3.354).in(Meters),
              Inches.of(12.341).in(Meters),
              //   new Rotation3d(Math.toRadians(0), Math.toRadians(-15), Math.toRadians(165))));
              new Rotation3d(Radians.zero(), Degrees.of(-15), Degrees.of(0))
                  .rotateBy(new Rotation3d(Radians.zero(), Degrees.of(0), Degrees.of(-165)))));

  public static final CameraConfig FRONT_LEFT_CAMERA =
      new CameraConfig(
          "front left",
          new Transform3d(
              Inches.of(13).in(Meters),
              Inches.of(11).in(Meters),
              Inches.of(3.875).in(Meters), // TODO get new
              //   new Rotation3d(0, Math.toRadians(-15), Math.toRadians(-30))));
              new Rotation3d(Radians.zero(), Degrees.of(-15), Degrees.of(0))
                  .rotateBy(new Rotation3d(Radians.zero(), Degrees.of(0), Degrees.of(-30)))));

  public static final CameraConfig FRONT_RIGHT_CAMERA =
      new CameraConfig(
          "front right",
          new Transform3d(
              Inches.of(13).in(Meters),
              Inches.of(-11).in(Meters),
              Inches.of(9.375).in(Meters),
              //   new Rotation3d(0, Math.toRadians(-15), Math.toRadians(30))));
              new Rotation3d(Radians.zero(), Degrees.of(-15), Degrees.of(0))
                  .rotateBy(new Rotation3d(Radians.zero(), Degrees.of(0), Degrees.of(30)))));

  // OV9281 constants for our configuration
  public static final int WIDTH = 1280;
  public static final int HEIGHT = 720;
  public static final Rotation2d FOV = Rotation2d.fromDegrees(55);

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1.5, 1.5, 7);
  public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 4);

  public static final double MAX_HEIGHT = 0.305;
  public static final double MAX_ANGLE = 0.3;
  public static final double MAX_AMBIGUITY = 0.2;

  // Total of 22 AprilTags
  // Reference: https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf (page 35)
  // Tag Locations (1-22)
  // Reef | Red: 6-11, Blue: 17-22
  // Cage | Red Side: 4-5, Blue Side: 14-15
  // Coral Station | Red Side: 1-2, Blue Side: 12-13
  // Processor | Red Side: 3 | Blue Side: 16

  public static final double[] TAG_WEIGHTS = {
    0.25, 0.25, 0.25, 0.25, 0.25, 1, 1, 1, 1, 1, 1, 0.25, 0.25, 0.25, 0.25, 0.25, 1, 1, 1, 1, 1, 1
  };
}
