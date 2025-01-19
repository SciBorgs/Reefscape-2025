package org.sciborgs1155.robot.coroller;

import static org.sciborgs1155.robot.Ports.GroundIntake.ROLLER_MOTOR;

import static org.sciborgs1155.robot.coroller.CorollerConstants.*;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

import com.ctre.phoenix6.hardware.TalonFX;

public class RealCoroller implements CorollerIO {
    private TalonFX motor;

    public RealCoroller() {
       motor = new TalonFX(ROLLER_MOTOR);

       motor.getConfigurator().apply(MOTOR_CONFIGURATION);

        FaultLogger.register(motor);
        TalonUtils.addMotor(motor);
    }

    @Override
    public void set(double power) {
        motor.set(power);
    }

    @Override
    public void close() {
        motor.close();
    }
}
