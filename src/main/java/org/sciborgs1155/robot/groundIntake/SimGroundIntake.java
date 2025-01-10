package org.sciborgs1155.robot.groundIntake;

import static org.sciborgs1155.robot.groundIntake.GroundIntakeConstants.STARTING_ANGLE;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.groundIntake.GroundIntakeConstants.*;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Simulated {@link GroundIntakeIO} class */
public class SimGroundIntake implements GroundIntakeIO {
    private final SingleJointedArmSim simulation = new SingleJointedArmSim(GEARBOX, GEARING, MOI, ARM_LENGTH.in(Meters), MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians), true, STARTING_ANGLE.getRadians());

    @Override
    public double position() {
        return simulation.getAngleRads();
    }

    @Override
    public double velocity() {
        return simulation.getVelocityRadPerSec();
    }

    @Override
    public void setArmVoltage(double voltage) {
        simulation.setInputVoltage(voltage);
    }

    /** Roller isn't simulated(This method does nothing) */
    @Override
    public void setRoller(double power) {

    }

}
