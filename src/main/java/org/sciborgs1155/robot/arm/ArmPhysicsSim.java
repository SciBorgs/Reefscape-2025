package org.sciborgs1155.robot.arm;

import edu.wpi.first.units.measure.Current;
import org.ironmaple.simulation.SimulatedArena;

public class ArmPhysicsSim extends SimulatedArena implements ArmIO {

  public ArmPhysicsSim() {
    super(null);
  }

  @Override
  public double position() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'position'");
  }

  @Override
  public double velocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'velocity'");
  }

  @Override
  public void setVoltage(double voltage) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }

  @Override
  public void setCurrentLimit(Current limit) {}

  @Override
  public void close() throws Exception {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'close'");
  }

  @Override
  public void placeGamePiecesOnField() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'placeGamePiecesOnField'");
  }

  @Override
  public void competitionPeriodic() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'competitionPeriodic'");
  }
}
