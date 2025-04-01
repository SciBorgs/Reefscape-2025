package org.sciborgs1155.robot.scoral;

import static org.sciborgs1155.robot.scoral.ScoralConstants.intakeSim;

public class SimCoral implements ScoralIO {
  @Override
  public boolean hasCoral() {
    return intakeSim.getGamePiecesAmount() != 0;
  }

  @Override
  public void close() throws Exception {}

  @Override
  public void scoralPower(double power, boolean outtaking) {
    if (power > 0 && outtaking) {
      intakeSim.setGamePiecesCount(0);
    } else if (power > 0 && !outtaking) {
      intakeSim.startIntake();
    } else {
      intakeSim.stopIntake();
    }
  }

  @Override
  public void algaePower(double power, boolean outtaking) {}
}
