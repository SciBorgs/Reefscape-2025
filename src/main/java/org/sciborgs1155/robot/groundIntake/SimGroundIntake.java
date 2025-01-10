package org.sciborgs1155.robot.groundIntake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Simulated {@link GroundIntakeIO} class */
public class SimGroundIntake implements GroundIntakeIO {
    private final SingleJointedArmSim simulation = new SingleJointedArmSim(DCMotor.getNEO(1),0, 0, 0, 0, 0, false, 0, null);
    
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
    public void set() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'set'");
    }
    
}
