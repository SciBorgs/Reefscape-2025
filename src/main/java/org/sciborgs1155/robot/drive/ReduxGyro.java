package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.ODOMETRY_PERIOD;
import static org.sciborgs1155.robot.Ports.Drive.CANANDGYRO;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.Queue;
import org.sciborgs1155.lib.FaultLogger;

/** GyroIO implementation for NavX */
public class ReduxGyro implements GyroIO {
  private final Canandgyro canandgyro = new Canandgyro(CANANDGYRO);

  private final Queue<Double> position;
  private final Queue<Double> timestamp;

  public ReduxGyro() {
    CanandgyroSettings settings =
        new CanandgyroSettings()
            .setAngularPositionFramePeriod(ODOMETRY_PERIOD.in(Seconds))
            .setAngularVelocityFramePeriod(ODOMETRY_PERIOD.in(Seconds));
    canandgyro.setSettings(settings, 0.25, 5);
    canandgyro.setYaw(0.0, 0.1);
    canandgyro.clearStickyFaults();

    FaultLogger.register(canandgyro);
    // See https://docs.reduxrobotics.com/canandgyro/programming/normal-operation#party-mode
    canandgyro.setPartyMode(0);

    position = TalonOdometryThread.getInstance().registerSignal(canandgyro::getYaw);
    timestamp = TalonOdometryThread.getInstance().makeTimestampQueue();
    canandgyro.clearStickyFaults();
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
  public double[][] odometryData() {
    Drive.lock.lock();
    try {
      double[][] data = {
        position.stream().mapToDouble((Double d) -> d).toArray(),
        timestamp.stream().mapToDouble((Double d) -> d).toArray()
      };
      position.clear();
      timestamp.clear();
      return data;
    } finally {
      Drive.lock.unlock();
    }
  }

  @Override
  public void reset() {
    canandgyro.setYaw(0);
  }

  @Override
  public void close() throws Exception {}
}
