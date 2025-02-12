package org.sciborgs1155.robot.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;
    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }
 
    @Override
    public double rate() {
        return gyroSimulation.getMeasuredAngularVelocity().magnitude();
    }
   
    @Override
    public void reset() {}
 
    @Override
    public Rotation2d rotation2d(){
        return gyroSimulation.getGyroReading();
    }

    @Override
    public Rotation3d rotation3d(){
            return null;
    }

    @Override
    public void close(){}
    
}

