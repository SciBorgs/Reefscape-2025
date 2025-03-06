package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class TransformUtils {

    /**
     * A transform that will translate the pose robot-relative right by a certain distance.
     * Negative distances will move the pose left.
     * @distance The distance that the pose will be moved.
     * @return A transform to strafe a pose.
     */
    public static Transform2d strafe(Distance distance) {
      return new Transform2d(
          new Translation2d(distance.in(Meters), Rotation2d.fromDegrees(-90)), Rotation2d.kZero);
    }

    /**
     * A transform that will translate the pose robot-relative forward by a certain distance.
     * Negative distances will move the pose backward.
     * @distance The distance that the pose will be moved.
     * @return A transform to move a pose forward.
     */
    public static Transform2d advance(Distance distance) {
        return new Transform2d(
            new Translation2d(distance.in(Meters), Rotation2d.fromDegrees(0)), Rotation2d.kZero);
      }
}
