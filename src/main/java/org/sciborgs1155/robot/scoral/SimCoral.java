package org.sciborgs1155.robot.scoral;

import static org.sciborgs1155.robot.scoral.ScoralConstants.intakeSim;

import org.ironmaple.simulation.SimulatedArena;

public class SimCoral implements ScoralIO {
    @Override
    public void set(double power) {
        if (power > 0 && hasCoral()) {
            // // TODO
            // intakeSim.setGamePiecesCount(0);
            // System.out.println(intakeSim.getGamePiecesAmount());
            System.out.println("has the Coral");
        } else if (power > 0 && !hasCoral()) {
            intakeSim.startIntake();
            System.out.println("test");
        } else {
            intakeSim.stopIntake();
            System.out.println("hmmmm");
        }
    }

    @Override
    public boolean hasCoral() {
        return intakeSim.getGamePiecesAmount() != 0;
    }

    @Override
    public void close() throws Exception {}
}
