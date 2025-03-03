package org.sciborgs1155.robot.drive;

import static org.sciborgs1155.robot.Ports.Drive.CANANDGYRO;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import org.sciborgs1155.lib.FaultLogger;

/** GyroIO implementation for NavX */
public class ReduxGyro implements GyroIO {
  private final Canandgyro canandgyro = new Canandgyro(CANANDGYRO);

  public ReduxGyro() {
    FaultLogger.register(canandgyro);
    // See https://docs.reduxrobotics.com/canandgyro/programming/normal-operation#party-mode
    canandgyro.setPartyMode(0);
  }

  @Override
  public double rate() {
    return canandgyro.getAngularVelocityYaw();
  }

  @Override
  public Rotation3d rotation3d() {
    return canandgyro.getRotation3d();
  }

  @Override
  public void reset() {
    canandgyro.setYaw(0);
  }

  @Override
  public void close() throws Exception {}

  @Override
  public Translation2d acceleration() {
    return new Translation2d(
        canandgyro.getAccelerationX(),
        canandgyro.getAccelerationY()); // .rotateBy(canandgyro.getRotation2d());

    // TODO We don't know if this is field relative or robot relative. if field relative add in the
    // commented code.
  }
}
