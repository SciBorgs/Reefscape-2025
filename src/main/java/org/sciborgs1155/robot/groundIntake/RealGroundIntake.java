package org.sciborgs1155.robot.groundIntake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

/** {@link GroundIntakeIO} class with {@link TalonFX} motor controllers */
public class RealGroundIntake implements GroundIntakeIO {
    /** Controls arm orientation */
    private final TalonFX pivotMotor;

    /** Controls intake power */
    private final TalonFX rollerMotor;

    @Override
    public double position() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'position'");
    }

    @Override
    public double velocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'velocity'");
    }

    @Override
    public void set() {
       
    }

    public RealGroundIntake(int pivotMotorID, int rollerMotorID) {
        pivotMotor = new TalonFX(pivotMotorID);
        rollerMotor = new TalonFX(rollerMotorID);

        // Resetting configurations
        pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
        rollerMotor.getConfigurator().apply(new TalonFXConfiguration());
    }

}
