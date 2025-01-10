package org.sciborgs1155.robot.groundIntake;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

import static org.sciborgs1155.robot.groundIntake.GroundIntakeConstants.*;

public class GroundIntake {

    private final GroundIntakeIO hardware;

    private final PIDController fb = new PIDController(kP, kI, kD);
    private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

    public static GroundIntake create() {
        return Robot.isReal() ? new GroundIntake(new RealGroundIntake()) : new GroundIntake(new SimGroundIntake());
    }

    public static GroundIntake none() {
    return new GroundIntake(new NoGroundIntake());
    }
    
    private GroundIntake(GroundIntakeIO hardware) {
        this.hardware = hardware;
    }

    

}
