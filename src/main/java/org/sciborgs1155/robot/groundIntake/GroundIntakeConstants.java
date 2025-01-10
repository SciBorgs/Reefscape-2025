package org.sciborgs1155.robot.groundIntake;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;

public class GroundIntakeConstants {
    
    //TODO not tuned (at all)
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0.01;

    public static final double kS = 1; //TODO do tese
    public static final double kV = 0;
    public static final double kA = 0.01;
    public static final double kG = 0.01;

    public static final Rotation2d INTAKE_ANGLE = Rotation2d.fromRadians(0); //TODO figure this out
    public static final Rotation2d OUTTAKE_ANGLE = Rotation2d.fromRadians(0); //TODO figure this one out too
    public static final Rotation2d STARTING_ANGLE = Rotation2d.fromRadians(0); //TODO ??????
    
}
