package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class BetterOdometry {

  private Translation2d fieldPosition;

  public BetterOdometry(Translation2d start) {
    fieldPosition = start;
  }

  public void resetOdometry(Translation2d pos) {
    fieldPosition = pos;
  }

  private static Translation2d moduleDisplacement(Translation2d v0, Translation2d v1) {
    Angle theta = v1.getAngle().minus(v0.getAngle()).getMeasure();
    Distance radius =
        Meters.of(
            2
                * PERIOD.in(Seconds)
                * (v0.getNorm() + 0.5 * (v1.getNorm() - v0.getNorm()))
                / theta.in(Radians));
    return new Translation2d(
        radius.times(2 * Math.sin(theta.div(2).in(Radians))).in(Meters),
        v0.getAngle().plus(Rotation2d.fromRadians(theta.div(2).in(Radians))));
  }

  public static Translation2d robotDisplacement(ModuleState[] states) {
    return Stream
  }

  public void update(Translation2d v1, Translation2d v2) {
  }

}
