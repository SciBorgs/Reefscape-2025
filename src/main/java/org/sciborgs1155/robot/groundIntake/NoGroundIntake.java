package org.sciborgs1155.robot.groundIntake;

/** Disfunctional Placeholder {@link GroundIntakeIO} */
public class NoGroundIntake implements GroundIntakeIO {
    @Override
    public double position() {
       return 0;
    }

    @Override
    public double velocity() {
      return 0;
    }

    @Override
    public void set() {}
    
}
