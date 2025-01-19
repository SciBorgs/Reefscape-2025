package org.sciborgs1155.robot.coroller;

public interface CorollerIO extends AutoCloseable {
    
    /**
     * Sets the power of the roller.
     * @param power The power of the roller to set it to; a number from -1 to 1.
     */
    public void set(double power);
}
