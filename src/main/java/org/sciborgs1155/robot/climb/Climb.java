package org.sciborgs1155.robot.climb;


import org.sciborgs1155.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase implements AutoCloseable {
    private final ClimbIO hardware;

    public static Climb create() {
        return new Climb(Robot.isReal() ? new RealClimb() : new NoClimb());
    }

    @Override
    public void close() throws Exception {
        
    }

    public Climb(ClimbIO hardware) {
        this.hardware = hardware;

        setDefaultCommand(runOnce(() -> hardware.setVoltage(0)).andThen(Commands.idle(this)));
    }
    


    
}
