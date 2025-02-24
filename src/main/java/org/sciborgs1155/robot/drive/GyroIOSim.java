package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
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
  public void reset() {}

  @Override
  public void close() throws Exception {
    throw new UnsupportedOperationException("Unimplemented method 'close'");
  }
}
