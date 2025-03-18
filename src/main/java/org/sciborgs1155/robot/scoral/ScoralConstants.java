package org.sciborgs1155.robot.scoral;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;
import static org.sciborgs1155.robot.drive.DriveConstants.driveSim;

import org.dyn4j.geometry.Triangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class ScoralConstants {
  public static final double SCORE_POWER = 0.85;
  public static final double INTAKE_POWER = 0.4;

  public static final Current STATOR_LIMIT = Amps.of(55);
  public static final Current CURRENT_LIMIT = Amps.of(50);

  public static final Time RAMP_TIME = Milliseconds.of(50);

  public static final IntakeSimulation intakeSim =
    new IntakeSimulation(
        "Coral",
        driveSim,
        new Triangle(new Vector2(0, 0), new Vector2(0.2, 0), new Vector2(0, 0.2)),
        1);
}
