package org.sciborgs1155.robot.hopper;

import org.sciborgs1155.robot.Ports.Hopper;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;

public class RealHopper implements HopperIO {
    private final SparkMax leftMotor = new SparkMax(Hopper.LEFT_MOTOR, MotorType.kBrushless); 
    private final SparkMax rightMotor = new SparkMax(Hopper.RIGHT_MOTOR, MotorType.kBrushless);
    private final DigitalInput beambreak = new DigitalInput(Hopper.BEAMBREAK);

    public RealHopper() {
        SparkBaseConfig config = new SparkMaxConfig(); // prob more configs to do
        config.inverted(true);

        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // one of the motors are inverted so that the motors can intake/outtake right
    }

    @Override
    public void setPower(double power) {
        leftMotor.set(power);
        rightMotor.set(power);
    }

    @Override
    public boolean beambreak() {
        return beambreak.get();
    }

    @Override
    public void close() {
        leftMotor.close();
        rightMotor.close();
        beambreak.close();
    }
}
