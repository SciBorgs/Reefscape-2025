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

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.sciborgs1155.robot.vision.Vision.CameraConfig;

public class VisionConstants {
  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  public static Rotation3d yawPitchRoll(
      double yawDegrees, double pitchDegrees, double rollDegrees) {
    return new Rotation3d(Degrees.of(rollDegrees), Degrees.of(pitchDegrees), Degrees.of(yawDegrees));
  }
  public static final CameraConfig FRONT_LEFT_CAMERA =
      new CameraConfig(
          "cam 0 RENAME",
          78,
          new Transform3d(
              Inches.of(11.935943),
              Inches.of(12.493204),
              Inches.of(5.176840 + 4.6),
              yawPitchRoll(65, -20, 180)),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);


  public static final CameraConfig FRONT_RIGHT_CAMERA =
      new CameraConfig(
          "cam 1 RENAME",
          78,
          new Transform3d(
              Inches.of(11.935943),
              Inches.of(-12.493204),
              Inches.of(5.176840 + 4.6),
              yawPitchRoll(-65, -20, 180)),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

  // ThriftyCam constants for our configuration
  public static final int WIDTH = 1280;
  public static final int HEIGHT = 720;
  public static final Rotation2d FOV = Rotation2d.fromDegrees(78);

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1.5, 1.5, 7);
  public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 4);

  public static final double MAX_HEIGHT = 0.305;
  public static final double MAX_ANGLE = 1;
  public static final double MAX_AMBIGUITY = 0.18;

  public static final double HENRYS_CONSTANT = 30;

  // Total of 22 AprilTags
  // Reference: https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf (page 35)
  // Tag Locations (1-22)
  // Reef | Red: 6-11, Blue: 17-22
  // Cage | Red Side: 4-5, Blue Side: 14-15
  // Coral Station | Red Side: 1-2, Blue Side: 12-13
  // Processor | Red Side: 3 | Blue Side: 16

  public static final double[] TAG_WEIGHTS = {
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
  };

  public static final Set<Integer> BAD_TAGS = Set.of();
}
