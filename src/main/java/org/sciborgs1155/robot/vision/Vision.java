package org.sciborgs1155.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static org.sciborgs1155.robot.vision.VisionConstants.FOV;
import static org.sciborgs1155.robot.vision.VisionConstants.FRONT_LEFT_CAMERA;
import static org.sciborgs1155.robot.vision.VisionConstants.FRONT_RIGHT_CAMERA;
import static org.sciborgs1155.robot.vision.VisionConstants.HEIGHT;
import static org.sciborgs1155.robot.vision.VisionConstants.MAX_AMBIGUITY;
import static org.sciborgs1155.robot.vision.VisionConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.vision.VisionConstants.MAX_HEIGHT;
import static org.sciborgs1155.robot.vision.VisionConstants.REEF_TAGS;
import static org.sciborgs1155.robot.vision.VisionConstants.TAG_LAYOUT;
import static org.sciborgs1155.robot.vision.VisionConstants.TAG_WEIGHTS;
import static org.sciborgs1155.robot.vision.VisionConstants.WIDTH;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.robot.FieldConstants;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.FieldConstants.Branch;

@Logged
public class Vision extends SubsystemBase{
  public static record CameraConfig(String name, Transform3d robotToCam) {}

  public static record PoseEstimate(EstimatedRobotPose estimatedPose, Matrix<N3, N1> standardDev) {}

  private final PhotonCamera[] cameras;
  // private final PhotonPoseEstimator[] estimators;
  // private final PhotonCameraSim[] simCameras;
  private final PhotonPipelineResult[] lastResults;
  private final Map<String, Boolean> camerasEnabled;
  // @Logged private final List<Pose3d> filteredEstimates;

  private VisionSystemSim visionSim;
  

  /** A factory to create new vision classes with our four configured cameras. */
  public static Vision create() {
    return Robot.isReal() ? new Vision(FRONT_RIGHT_CAMERA, FRONT_LEFT_CAMERA) : new Vision();
  }

  public static Vision none() {
    return new Vision();
  }

  public Vision(CameraConfig... configs) {
    cameras = new PhotonCamera[configs.length];
    // estimators = new PhotonPoseEstimator[configs.length];
    // simCameras = new PhotonCameraSim[configs.length];
    lastResults = new PhotonPipelineResult[configs.length];
    // filteredEstimates = new ArrayList<>();
    camerasEnabled = new HashMap<>();

    for (int i = 0; i < configs.length; i++) {
      PhotonCamera camera = new PhotonCamera(configs[i].name());
      // PhotonPoseEstimator estimator =
      //     new PhotonPoseEstimator(
      //         VisionConstants.TAG_LAYOUT,
      //         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      //         configs[i].robotToCam());

      // estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      cameras[i] = camera;
      // estimators[i] = estimator;
      lastResults[i] = new PhotonPipelineResult();
      camerasEnabled.put(camera.getName(), true);

      FaultLogger.register(camera);
    }
  }

 
  public void disableCam(String name) {
    camerasEnabled.put(name, false);
  }

  public void enableCam(String name) {
    camerasEnabled.put(name, true);
  }

  public boolean getCameraStatus(String name) {
    return camerasEnabled.get(name);
  }

  /**
   * Returns the poses of all currently visible tags.
   *
   * @return An array of Pose3ds.
   */
  public Pose3d[] getSeenTags() {
    return Arrays.stream(lastResults)
        .flatMap(c -> c.targets.stream())
        .filter(t -> t.getPoseAmbiguity() < MAX_AMBIGUITY)
        .map(PhotonTrackedTarget::getFiducialId)
        .map(TAG_LAYOUT::getTagPose)
        .map(Optional::get)
        .toArray(Pose3d[]::new);
  }

  

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> estimationStdDevs(
      Pose2d estimatedPose, PhotonPipelineResult pipelineResult) {
    var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
    var targets = pipelineResult.getTargets();
    double avgDist = 0;
    double avgWeight = 0;
    for (var tgt : targets) {
      var tagPose = TAG_LAYOUT.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
      avgWeight += TAG_WEIGHTS[tgt.getFiducialId() - 1];
    }
    if (targets.size() == 0) return estStdDevs;

    avgDist /= targets.size();
    avgWeight /= targets.size();

    // Decrease std devs if multiple targets are visible
    if (targets.size() > 1) estStdDevs = VisionConstants.MULTIPLE_TAG_STD_DEVS;
    // Increase std devs based on (average) distance
    if (targets.size() == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    // disregard estimate heading after initial reposition
    if (DriverStation.isEnabled()) estStdDevs.set(2, 0, Double.MAX_VALUE);

    return estStdDevs.times(avgWeight);
  }

  @NotLogged
  public Transform3d[] cameraTransforms() {
    return new Transform3d[] {
      FRONT_LEFT_CAMERA.robotToCam(), FRONT_RIGHT_CAMERA.robotToCam(),
    };
  }
   
  public Command directTagAlignment(Drive drive){
    double tagYaw;
    boolean targetSeen = false;
    for (int i = 0; i < lastResults.length; i++) {
      if (lastResults[i].hasTargets()){
        for (var target : getSeenTags()){
          tagYaw = target.getRotation().getMeasureX().in(Degrees);
          targetSeen = true;
        }
      }
    }
    return run(drive.driveRobotRelative(() -> 0, () -> 0, ()))
  }
  /**
   * Updates the vision field simulation. This method should not be called when code is running on
   * the robot.
   */
  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }
}
