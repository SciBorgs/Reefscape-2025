package org.sciborgs1155.robot.groundIntake;

import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.groundIntake.GroundIntakeConstants.*;

public class GroundIntake extends SubsystemBase implements Logged, AutoCloseable {

    private final GroundIntakeIO hardware;

    private final PIDController fb = new PIDController(kP, kI, kD);
    private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

    /**
     * Creates a new GroundIntake, which will have real hardware if the robot is real, and simulated if it isn't.
     * @return
     */
    public static GroundIntake create() {
        return Robot.isReal() ? new GroundIntake(new RealGroundIntake()) : new GroundIntake(new SimGroundIntake());
    }

    /**
     * Creates a new GroundIntake with no hardware.
     * @return A new GroundIntake to... not do anything.
     */
    public static GroundIntake none() {
    return new GroundIntake(new NoGroundIntake());
    }
    
    /**
     * Constructor.
     * @param hardware A GroundIntakeIO that will act as the hardware.
     */
    private GroundIntake(GroundIntakeIO hardware) {
        this.hardware = hardware;
    }

    /**
     * Runs the roller intake forward.
     */
    public void rollerIntake() {
        hardware.setRoller(INTAKE_POWER);
    }

    /**
     * Runs the roller intake backwards.
     */
    public void rollerOuttake() {
        hardware.setRoller(OUTTAKE_POWER);
    }

    public Command setArm(double goal) {
        double feedForward = ff.calculate(hardware.position(), hardware.velocity());
        double feedBack = fb.calculate(hardware.position(), goal);
        return run(() -> hardware.setArmVoltage(feedBack + feedForward));
    }

    @Override
    public void close() {
        //TODO put close() into io methods and then implement this one
    }

    

    

}
