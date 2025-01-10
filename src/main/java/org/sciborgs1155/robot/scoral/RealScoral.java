package org.sciborgs1155.robot.scoral;

import org.sciborgs1155.robot.Ports;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class RealScoral implements ScoralIO{

    private final SparkMax topMotor;
    private final SparkMax bottomMotor;
    
    private final DigitalInput beambreak; 

    public RealScoral(){
        topMotor = new SparkMax(Ports.Scoral.TOP_ROLLER, MotorType.kBrushed);
        bottomMotor = new SparkMax(Ports.Scoral.BOTTOM_ROLLER, MotorType.kBrushed);

        beambreak = new DigitalInput(Ports.Scoral.BEAMBREAK);

        

    }

    @Override
    public void outake() {
        // TODO : SCORAL OUTTAKE
    }

    @Override
    public boolean beambreak() {
        return beambreak.get();
    }
    
}
