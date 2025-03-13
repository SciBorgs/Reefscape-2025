package org.sciborgs1155.robot.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N2;
import org.sciborgs1155.lib.FaultLogger;

/** GyroIO implementation for NavX */
public class NavXGyro implements GyroIO {
  private final AHRS ahrs = new AHRS(NavXComType.kMXP_SPI);

  public NavXGyro() {
    FaultLogger.register(ahrs);
  }

  @Override
  public double rate() {
    return ahrs.getRate();
  }

  @Override
  public Rotation3d rotation3d() {
    return ahrs.getRotation3d();
  }

  @Override
  public Vector<N2> acceleration() {
    return VecBuilder.fill(ahrs.getWorldLinearAccelX(), ahrs.getWorldLinearAccelY());
  }

  @Override
  public void reset() {
    ahrs.reset();
  }

  @Override
  public void close() throws Exception {}
}
