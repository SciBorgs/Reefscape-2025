package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.drive.DriveConstants.driveSim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class SimGyro implements GyroIO {
  private final GyroSimulation gyroSimulation;
  private ChassisSpeeds speeds = new ChassisSpeeds();

  public SimGyro(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public double rate() {
    return this.gyroSimulation.getMeasuredAngularVelocity().magnitude();
  }

  @Override
  public Rotation2d rotation2d() {
    return this.gyroSimulation.getGyroReading();
  }

  @Override
  public Rotation3d rotation3d() {
    Rotation3d rotation3d = new Rotation3d();
    return rotation3d;
  }

  @Override
  public double[][] odometryData() {
    return new double[20][20];
  }

  @Override
  public void reset() {}

  @Override
  public void close() throws Exception {}

  @Override
  public Vector<N2> acceleration() {
    ChassisSpeeds newSpeeds = driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    Vector<N2> accel =
        VecBuilder.fill(
            (newSpeeds.vxMetersPerSecond - speeds.vxMetersPerSecond) / PERIOD.in(Seconds),
            (newSpeeds.vyMetersPerSecond - speeds.vyMetersPerSecond) / PERIOD.in(Seconds));
    speeds = newSpeeds;
    return accel;
  }
}
