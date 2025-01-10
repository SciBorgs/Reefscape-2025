package org.sciborgs1155.robot.groundIntake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import static org.sciborgs1155.robot.Ports.GroundIntake.*;

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


    public RealGroundIntake() {
        pivotMotor = new TalonFX(ARM_MOTOR);
        rollerMotor = new TalonFX(ROLLER_MOTOR);

        // Resetting configurations
        pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
        rollerMotor.getConfigurator().apply(new TalonFXConfiguration());
    }

    @Override
    public void setArmVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setArmVoltage'");
    }

    @Override
    public void setRoller(double power) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setRoller'");
    }

}
