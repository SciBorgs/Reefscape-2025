package org.sciborgs1155.robot.vision;

import static org.sciborgs1155.robot.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import monologue.Annotations.Log;
import monologue.Logged;
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
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.robot.FieldConstants;
import org.sciborgs1155.robot.Robot;

public class Vision implements Logged {
  public static record CameraConfig(String name, Transform3d robotToCam) {}

  public static record PoseEstimate(EstimatedRobotPose estimatedPose, Matrix<N3, N1> standardDev) {}

  private final PhotonCamera[] cameras;
  private final PhotonPoseEstimator[] estimators;
  private final PhotonCameraSim[] simCameras;
  private final PhotonPipelineResult[] lastResults;
  private final Map<String, Boolean> camerasEnabled;

  private VisionSystemSim visionSim;

  /** A factory to create new vision classes with our four configured cameras. */
  public static Vision create() {
    return Robot.isReal()
        ? new Vision(
            FRONT_LEFT_CAMERA,
            FRONT_RIGHT_CAMERA,
            BACK_MIDDLE_CAMERA,
            BACK_LEFT_CAMERA,
            BACK_RIGHT_CAMERA)
        : new Vision();
  }

  public static Vision none() {
    return new Vision();
  }

  public Vision(CameraConfig... configs) {
    cameras = new PhotonCamera[configs.length];
    estimators = new PhotonPoseEstimator[configs.length];
    simCameras = new PhotonCameraSim[configs.length];
    lastResults = new PhotonPipelineResult[configs.length];
    camerasEnabled = new HashMap<>();

    for (int i = 0; i < configs.length; i++) {
      PhotonCamera camera = new PhotonCamera(configs[i].name());
      PhotonPoseEstimator estimator =
          new PhotonPoseEstimator(
              VisionConstants.TAG_LAYOUT,
              PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
              configs[i].robotToCam());

      estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      cameras[i] = camera;
      estimators[i] = estimator;
      lastResults[i] = new PhotonPipelineResult();
      camerasEnabled.put(camera.getName(), true);

      FaultLogger.register(camera);
    }

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(VisionConstants.TAG_LAYOUT);

      for (int i = 0; i < cameras.length; i++) {
        var prop = new SimCameraProperties();
        prop.setCalibration(WIDTH, HEIGHT, FOV);
        prop.setCalibError(0.15, 0.05);
        prop.setFPS(45);
        prop.setAvgLatencyMs(12);
        prop.setLatencyStdDevMs(3.5);

        PhotonCameraSim cameraSim = new PhotonCameraSim(cameras[i], prop);
        cameraSim.setMaxSightRange(5);
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, configs[i].robotToCam());
        simCameras[i] = cameraSim;
      }
    }
  }

  public void logCamEnabled() {
    camerasEnabled.forEach((name, enabled) -> log(name + " enabled", enabled));
  }

  /**
   * Returns a list of all currently visible pose estimates and their standard deviation vectors.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public PoseEstimate[] estimatedGlobalPoses() {
    List<PoseEstimate> estimates = new ArrayList<>();
    for (int i = 0; i < estimators.length; i++) {
      if (camerasEnabled.get(cameras[i].getName())) {
        var unreadChanges = cameras[i].getAllUnreadResults();

        String name = cameras[i].getName();

        Optional<EstimatedRobotPose> estimate = Optional.empty();

        int unreadLength = unreadChanges.size();

        // feeds latest result for visualization; multiple different pos breaks getSeenTags()
        lastResults[i] = unreadLength == 0 ? lastResults[i] : unreadChanges.get(unreadLength - 1);

        for (int j = 0; j < unreadLength; j++) {
          var change = unreadChanges.get(j);

          // only reef tags
          if (Set.of("back left", "back right").contains(name)) {
            change.targets =
                change.targets.stream().filter(t -> REEF_TAGS.contains(t.fiducialId)).toList();
            change.multitagResult =
                change.multitagResult.filter(
                    r ->
                        r.fiducialIDsUsed.stream()
                            .map(id -> REEF_TAGS.contains((int) id))
                            .reduce(true, (a, b) -> a && b));
          }

          // negate pitch
          if (cameras[i].getName() != "back middle") {
            change.targets.stream()
                .forEach(
                    t -> {
                      t.pitch = -t.pitch;
                    });
            change.multitagResult =
                change.multitagResult.filter(
                    r ->
                        r.fiducialIDsUsed.stream()
                            .map(id -> REEF_TAGS.contains((int) id))
                            .reduce(true, (a, b) -> a && b));
          }

          // remove ambiguity
          change.targets =
              change.targets.stream().filter(t -> t.poseAmbiguity < MAX_AMBIGUITY).toList();
          change.multitagResult =
              change.multitagResult.filter(r -> r.estimatedPose.ambiguity < MAX_AMBIGUITY);

          estimate = estimators[i].update(change);

          log(name + " estimates present", estimate.isPresent());
          estimate
              .filter(
                  f -> {
                    boolean valid =
                        FieldConstants.inField(f.estimatedPose)
                            && Math.abs(f.estimatedPose.getZ()) < MAX_HEIGHT
                            && Math.abs(f.estimatedPose.getRotation().getX()) < MAX_ANGLE
                            && Math.abs(f.estimatedPose.getRotation().getY()) < MAX_ANGLE;
                    if (!valid) {
                      log(name + " rejected estimate", f.estimatedPose);
                      FaultLogger.report(name, "Estimate outside field!", FaultType.INFO);
                    }
                    return valid;
                  })
              .ifPresent(
                  e ->
                      estimates.add(
                          new PoseEstimate(
                              e, estimationStdDevs(e.estimatedPose.toPose2d(), change))));
        }
      }
    }
    return estimates.toArray(PoseEstimate[]::new);
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

  public void feedEstimatorHeading(Rotation2d heading) {
    for (PhotonPoseEstimator estimator : estimators) {
      estimator.addHeadingData(Timer.getFPGATimestamp(), heading);
    }
  }

  /**
   * Returns the poses of all currently visible tags.
   *
   * @return An array of Pose3ds.
   */
  @Log.NT
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

    return estStdDevs.times(avgWeight);
  }

  public Transform3d[] cameraTransforms() {
    return new Transform3d[] {
      FRONT_LEFT_CAMERA.robotToCam(),
      FRONT_RIGHT_CAMERA.robotToCam(),
      BACK_LEFT_CAMERA.robotToCam(),
      BACK_RIGHT_CAMERA.robotToCam(),
      BACK_MIDDLE_CAMERA.robotToCam()
    };
  }

  /**
   * Updates the vision field simulation. This method should not be called when code is running on
   * the robot.
   */
  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }
}
