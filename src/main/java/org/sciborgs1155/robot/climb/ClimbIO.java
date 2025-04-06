package org.sciborgs1155.robot.climb;

public interface ClimbIO extends AutoCloseable {
    /**
     * @return The position in radians.
     */
    public double position();
  
    /**
     * @return The position in radians/sec.
     */
    public double velocity();
  
    /** Sets the voltage of the arm motor. */
    public void setVoltage(double voltage);
  }
