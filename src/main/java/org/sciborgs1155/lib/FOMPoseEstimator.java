package org.sciborgs1155.lib;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Arrays;
import java.util.function.Function;
import org.sciborgs1155.robot.vision.Vision.PoseEstimate;

public class FOMPoseEstimator {
  private Pose2d robotPose;
  private final SwerveDrivePoseEstimator odometry;

  public static record Estimate(double measurement, double FOM) {}

  public static record OdometryUpdate(
      double timestamp, Rotation2d angle, SwerveModulePosition[] modulePositions) {}

  public FOMPoseEstimator(
      Pose2d initialPose,
      SwerveDriveKinematics kinematics,
      SwerveModulePosition[] modulePositions) {
    this.robotPose = initialPose;
    // this.odometry = odometry;
    this.odometry =
        new SwerveDrivePoseEstimator(
            kinematics, initialPose.getRotation(), modulePositions, initialPose);
  }

  public void updateNoVision(Rotation2d angle, SwerveModulePosition[] modulePositions) {
    odometry.update(angle, modulePositions);
    robotPose = odometry.getEstimatedPosition();
  }

  public void update(
      OdometryUpdate[] odometryUpdates, Vector<N3> odometryFOM, PoseEstimate... visionEstimates) {
    Pose2d lastOdometryPose = odometry.getEstimatedPosition();

    for (OdometryUpdate update : odometryUpdates) {
      odometry.updateWithTime(update.timestamp, update.angle, update.modulePositions);
    }

    Pose2d odometryEstimate =
        robotPose.plus(odometry.getEstimatedPosition().minus(lastOdometryPose));

    Estimate[] xEstimates = new Estimate[visionEstimates.length + 1];
    Estimate[] yEstimates = new Estimate[visionEstimates.length + 1];
    Estimate[] thetaEstimates = new Estimate[visionEstimates.length + 1];

    xEstimates[0] = new Estimate(odometryEstimate.getX(), odometryFOM.get(0));
    yEstimates[0] = new Estimate(odometryEstimate.getY(), odometryFOM.get(1));
    thetaEstimates[0] =
        new Estimate(odometryEstimate.getRotation().getRadians(), odometryFOM.get(0));

    for (int i = 0; i < visionEstimates.length; i++) {
      PoseEstimate visionEstimate = visionEstimates[i];

      Pose2d visionPose = visionEstimate.estimatedPose().estimatedPose.toPose2d();
      Matrix<N3, N1> stdevs = visionEstimate.standardDev();
      double minAmbiguity =
          visionEstimate.estimatedPose().targetsUsed.stream()
              .mapToDouble(t -> t.poseAmbiguity)
              .min()
              .orElse(1);
      Matrix<N3, N1> visionFOMs =
          stdevs.times(minAmbiguity == 1 ? Double.MAX_VALUE : 1 / (1 - minAmbiguity));

      xEstimates[i + 1] = new Estimate(visionPose.getX(), visionFOMs.get(0, 0));

      yEstimates[i + 1] = new Estimate(visionPose.getX(), visionFOMs.get(1, 0));
      thetaEstimates[i + 1] = new Estimate(visionPose.getX(), visionFOMs.get(2, 0));
    }

    robotPose =
        new Pose2d(
            newEstimate(xEstimates),
            newEstimate(yEstimates),
            Rotation2d.fromRadians(newEstimate(thetaEstimates)));
  }

  public Pose2d pose() {
    return robotPose;
  }

  public static double newEstimate(Estimate... estimates) {
    Function<Estimate, Double> inverseSquare =
        e -> e.FOM == 0 ? Double.MAX_VALUE : 1 / Math.pow(e.FOM, 2);
    return Arrays.stream(estimates).mapToDouble(e -> inverseSquare.apply(e) * e.measurement).sum()
        / Arrays.stream(estimates).mapToDouble(e -> inverseSquare.apply(e)).sum();
  }

  public void resetPose(Pose2d pose) {
    robotPose = pose;
    odometry.resetPose(pose);
  }
}
