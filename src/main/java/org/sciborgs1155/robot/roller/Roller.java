package org.sciborgs1155.robot.roller;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

import static org.sciborgs1155.robot.roller.RollerConstants.*;

public class Roller extends SubsystemBase implements Logged, AutoCloseable{

    private final RollerIO hardware;

    public Roller create() {
        return Robot.isReal() ? new Roller(new RealRoller()) : new Roller(new NoRoller());
    }

    public Roller none() {
        return new Roller(new NoRoller());
    }

    private Roller(RollerIO hardware) {
        this.hardware = hardware;
    }

    /**
     * Runs the roller intake forward.
     */
    public Command intake() {
        return run(() -> hardware.set(INTAKE_POWER));
    }

    /**
     * Runs the roller intake backwards.
     */
    public void outtake() {
        hardware.set(OUTTAKE_POWER);
    }

    @Override
    public void close() {
        hardware.close();
    }
    
}
