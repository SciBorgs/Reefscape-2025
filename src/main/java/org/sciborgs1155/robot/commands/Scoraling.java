package org.sciborgs1155.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;
import monologue.Annotations.Log;

import java.util.Set;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.led.LEDStrip;
import org.sciborgs1155.robot.scoral.Scoral;

public class Scoraling implements Logged {
  private final Hopper hopper;
  private final Scoral scoral;
  private final Elevator elevator;
  private final LEDStrip leftStrip;
  private final LEDStrip rightStrip;
  private final Trigger stopHPI;

  @Log.NT
  public boolean stopHPI() {
    return stopHPI.getAsBoolean();
  }

  public Scoraling(
      Hopper hopper, Scoral scoral, Elevator elevator, LEDStrip leftStrip, LEDStrip rightStrip) {
    this.hopper = hopper;
    this.scoral = scoral;
    this.elevator = elevator;
    this.leftStrip = leftStrip;
    this.rightStrip = rightStrip;

    /*
    Causes the intaking command to end if the coral reaches the desired state between the hps and scoral
    beambreaks.
    */
    stopHPI = hopper
        .beambreakTrigger
        .negate()
        .or(scoral.beambreakTrigger)
        .onFalse(Commands.runOnce(() -> stop = true));
            // stop()
            //     .alongWith(leftStrip.blink(Color.kWhite), rightStrip.blink(Color.kWhite))
            //     .onlyIf(
            //         () ->
            //             hopper.getCurrentCommand().getName().equals("intakingHPS")
            //                 && scoral.getCurrentCommand().getName().equals("intakingHPS")));
  }

  @Log.NT
  private boolean stop = false;

  /** A command which intakes from the human player station. */
  public Command hpsIntake() {
    return  Commands.runOnce(() -> stop = false).andThen(
    
    elevator
                    .retract()
                    .alongWith(
                        Commands.waitUntil(elevator::atGoal)
                    .andThen(runRollers()))
                    .until(() -> stop)
                    .finallyDo(() -> stop = false) )      
        .withName("intakingHPS");
  }

  /**
   * A command which scores a coral at the given level, assuming you are already at the correct
   * branch.
   *
   * @param level the level the scoral scores in
   */
  public Command scoral(Level level) {
    return elevator
        .scoreLevel(level)
        .alongWith(Commands.waitUntil(elevator::atGoal).andThen(scoral.score()))
        .withName("scoraling");
  }

  /**
   * A command which grabs the algae from above the level given; only L2 and L3 are allowed.
   *
   * @param level the level to score above
   */
  public Command cleanAlgae(Level level) {
    return elevator
        .clean(level)
        .alongWith(Commands.waitUntil(elevator::atGoal).andThen(scoral.score()))
        .onlyIf(scoral.beambreakTrigger)
        .withName("cleanAlgae");
  }

  /** A command which halts both the hopper and the scoral. */
  public Command stop() {
    return hopper.stop().alongWith(scoral.stop()).withName("stopping");
  }

  /**
   * A command which runs the hps + scoral rollers forward (generally as a form of intaking), then
   * runs them back and forth until
   */
  public Command runRollers() {
    return hopper
        .intake()
        .alongWith(scoral.intake())
        .withTimeout(1)
        .andThen(
            (runRollersBack().withTimeout(0.2)//.andThen(runRollers())
                .onlyIf(hopper.beambreakTrigger.negate())))
        .withName("runningRollers");
  }

  /** A command which runs the hps + scoral rollers forward (generally as a form of intaking). */
  public Command runRollersBack() {
    return hopper.outtake().alongWith(scoral.algae()).withName("runningRollers");
  }
}
