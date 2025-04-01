package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MIN_EXTENSION;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorVisualizer {
  @NotLogged private final Mechanism2d mech;
  private final MechanismLigament2d elevator;
  private final String name;

  public ElevatorVisualizer(String name, Color8Bit color) {
    this.name = name;
    mech = new Mechanism2d(50, 50);
    MechanismRoot2d chassis = mech.getRoot("chassis", 25, 10);
    elevator =
        chassis.append(
            new MechanismLigament2d("elevator", MIN_EXTENSION.in(Meters) * 10, 90, 3, color));
  }

  /**
   * Sets the length of the elevator visualization.
   *
   * @param length The length to set.
   */
  public void setLength(double length) {
    elevator.setLength(length * 10);
    SmartDashboard.putData("Robot/elevator/" + name, mech);
  }
}
