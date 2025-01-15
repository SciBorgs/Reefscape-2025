package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.Constants.Field.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import org.sciborgs1155.robot.Constants.Field.Branch;
import org.sciborgs1155.robot.Constants.Field.Level;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.scoral.Scoral;

public class Alignment {
  private Drive drive;
  private Elevator elevator;
  private Scoral scoral;

  /**
   * Constructor for an Alignment command object.
   *
   * @param drive The operated drivetrain.
   * @param elevator The operated elevator.
   */
  public Alignment(Drive drive, Elevator elevator, Scoral scoral) {
    this.drive = drive;
    this.elevator = elevator;
    this.scoral = scoral;
  }

  public Command reef(Level level, Branch branch) {
    return drive
        .driveTo(branch.pose)
        .alongWith(elevator.scoreLevel(level).until(() -> elevator.atPosition(level.height)))
        .andThen(scoral.outtake());
  }

  public Command safeReef(Level level, Branch branch) {
    return drive
        .driveTo(branch.pose)
        .andThen(elevator.scoreLevel(level).until(() -> elevator.atPosition(level.height)))
        .andThen(scoral.outtake());
  }

  public Command reef(Level level) {
    return drive
        .driveTo(nearestReef(drive.pose()))
        .alongWith(elevator.scoreLevel(level).until(() -> elevator.atPosition(level.height)))
        .andThen(scoral.outtake());
  }

  public Command source() {
    return drive
        .driveTo(drive.pose().nearest(List.of(LEFT_SOURCE, RIGHT_SOURCE)))
        .alongWith(elevator.retract());
  }

  public Command processor() {
    return drive.driveTo(PROCESSOR);
  }

  public Command cage() {
    return drive.driveTo(nearestCage(drive.pose()));
  }
}
