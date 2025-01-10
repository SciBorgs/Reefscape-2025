package org.sciborgs1155.robot.scoral;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Scoral extends SubsystemBase{

    public Scoral(ScoralIO scoral) {
        System.out.println("");
    }

    public static Scoral create(){
        return Robot.isReal() ? new Scoral(new RealScoral()) : new Scoral(new SimScoral());
    }

    public static Scoral none(){
        return new Scoral(new NoScoral());
    }
}
