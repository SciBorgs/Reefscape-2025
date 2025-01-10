package org.sciborgs1155.robot.groundIntake;

/** Hardware interface for {@link GroundIntake} subsystem */
public interface GroundIntakeIO {

    /**
     * @return The position in radians.
     */
    public double position();

    /**
     * @return The position in radians/sec.
     */
    public double velocity();

    /**
     * Sets the voltage of the arm motor.
     */
    public void set();
}
