package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MIN_HEIGHT;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import monologue.Annotations.Log;
import monologue.Logged;

public class ElevatorVisualizer implements Logged {
  @Log.NT private final Mechanism2d mech;
  private final MechanismLigament2d elevator;

  public ElevatorVisualizer(Color8Bit color) {
    mech = new Mechanism2d(50, 50);
    MechanismRoot2d chassis = mech.getRoot("chassis", 25, 10);
    elevator =
        chassis.append(
            new MechanismLigament2d("elevator", MIN_HEIGHT.in(Meters) * 10, 90, 3, color));
  }

  /**
   * Sets the length of the elevator visualization.
   *
   * @param length The length to set.
   */
  public void setLength(double length) {
    elevator.setLength(length * 10);
  }
}
