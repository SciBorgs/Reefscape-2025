
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
import java.util.Set;
import org.sciborgs1155.robot.vision.Vision.CameraConfig;

public class VisionConstants {
  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final CameraConfig BACK_LEFT_CAMERA =
      new CameraConfig(
          "back left",
          new Transform3d(
              Inches.of(-13.227812).in(Meters),
              Inches.of(12.049788).in(Meters),
              Inches.of(9.629971).in(Meters),
              new Rotation3d(Radians.zero(), Degrees.of(-25), Radians.zero())
                  .rotateBy(new Rotation3d(Radians.zero(), Radians.zero(), Degrees.of(160)))));

  public static final CameraConfig BACK_RIGHT_CAMERA =
      new CameraConfig(
          "back right",
          new Transform3d(
              Inches.of(-13.227812).in(Meters),
              Inches.of(-12.049788).in(Meters),
              Inches.of(9.629971).in(Meters),
              new Rotation3d(Radians.zero(), Degrees.of(-25), Degrees.of(0))
                  .rotateBy(new Rotation3d(Radians.zero(), Degrees.of(0), Degrees.of(-160)))));

  public static final CameraConfig BACK_MIDDLE_CAMERA =
      new CameraConfig(
          "back middle",
          new Transform3d(
              Inches.of(-9.340270).in(Meters), // -10
              Inches.of(4.449673).in(Meters), // 3.354
              Inches.of(12.249288).in(Meters), // 12.341
              new Rotation3d(Radians.zero(), Degrees.of(-15), Degrees.of(0))
                  .rotateBy(new Rotation3d(Radians.zero(), Degrees.of(0), Degrees.of(-165)))));

  public static final CameraConfig FRONT_LEFT_CAMERA =
      new CameraConfig(
          "front left",
          new Transform3d(
              Inches.of(12.931292).in(Meters),
              Inches.of(10.883863).in(Meters),
              Inches.of(8.739474).in(Meters),
              new Rotation3d(Radians.zero(), Degrees.of(-17.5), Degrees.of(0))
                  .rotateBy(new Rotation3d(Radians.zero(), Degrees.of(0), Degrees.of(-35)))));

  public static final CameraConfig FRONT_RIGHT_CAMERA =
      new CameraConfig(
          "front right",
          new Transform3d(
              Inches.of(12.931292).in(Meters),
              Inches.of(-10.883863).in(Meters),
              Inches.of(8.739474).in(Meters),
              new Rotation3d(Radians.zero(), Degrees.of(-17.5), Degrees.of(0))
                  .rotateBy(new Rotation3d(Radians.zero(), Degrees.of(0), Degrees.of(35)))));

  // ThriftyCam constants for our configuration
  public static final int WIDTH = 1280;
  public static final int HEIGHT = 720;
  public static final Rotation2d FOV = Rotation2d.fromDegrees(80);

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1.5, 1.5, 7);
  public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 4);

  public static final double MAX_HEIGHT = 0.305;
  public static final double MAX_ANGLE = 1;
  public static final double MAX_AMBIGUITY = 0.18;

  // Total of 22 AprilTags
  // Reference: https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf (page 35)
  // Tag Locations (1-22)
  // Reef | Red: 6-11, Blue: 17-22
  // Cage | Red Side: 4-5, Blue Side: 14-15
  // Coral Station | Red Side: 1-2, Blue Side: 12-13
  // Processor | Red Side: 3 | Blue Side: 16

  public static final double[] TAG_WEIGHTS = {
    0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1
  };

  public static final Set<Integer> REEF_TAGS = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 22);
}
