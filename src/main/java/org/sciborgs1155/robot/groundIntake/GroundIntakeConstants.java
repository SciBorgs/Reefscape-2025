package org.sciborgs1155.robot.groundIntake;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class GroundIntakeConstants {
    // TODO not tuned (at all)
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0.01;

    public static final double kS = 1; // TODO do tese
    public static final double kV = 0;
    public static final double kA = 0.01;
    public static final double kG = 0.01;

    public static final Rotation2d INTAKE_ANGLE = Rotation2d.fromRadians(0); //TODO figure this out
    public static final Rotation2d OUTTAKE_ANGLE = Rotation2d.fromRadians(0); //TODO figure this one out too
    public static final Rotation2d STARTING_ANGLE = Rotation2d.fromRadians(0); //TODO ??????

    // The amount of power going into the roller (from 0-1)
    public static final double INTAKE_POWER = 0.5; //TODO arbitrary value
    public static final double OUTTAKE_POWER = -0.2; //TODO also arbitrary

    public static final DCMotor GEARBOX = DCMotor.getNEO(1);
    public static final double GEARING = 8.21;
    public static final double MOI = 1;

    public static final Distance ARM_LENGTH = Centimeters.of(10);
    public static final Angle MIN_ANGLE = Radians.of(0.5);
    public static final Angle MAX_ANGLE = Radians.of(3);
}