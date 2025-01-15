package org.sciborgs1155.robot.commands;

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

  /**
   * Drives to a designated reef branch while raising the elevator, and then scores onto a designated level on that branch.
   * @param level The level (L1, L2, L3, L4) being scored on.
   * @param branch The reef branch (A, B, C, etc.) being scored on.
   * @return A command to quickly prepare and then score in the reef.
   */
  public Command reef(Level level, Branch branch) {
    return drive
        .driveTo(branch.pose)
        .alongWith(elevator.scoreLevel(level).until(() -> elevator.atPosition(level.height)))
        .andThen(scoral.outtake());
  }

  /**
   * Drives to a designated reef branch, then raises the elevator, and then scores onto a designated level on that branch.
   * @param level The level (L1, L2, L3, L4) being scored on.
   * @param branch The reef branch (A, B, C, etc.) being scored on.
   * @return A command to score in the reef without raising the elevator while moving.
   */
  public Command safeReef(Level level, Branch branch) {
    return drive
        .driveTo(branch.pose)
        .andThen(elevator.scoreLevel(level).until(() -> elevator.atPosition(level.height)))
        .andThen(scoral.outtake());
  }

  /**
   * Drives to the nearest reef branch while raising the elevator, and then scores
   * @param level
   * @return
   */
  public Command reef(Level level) {
    return reef(level, Branch.nearest(drive.pose()));
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
