package org.sciborgs1155.lib;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.FaultLogger.FaultType;

public class TalonUtils implements Logged {
  private static final Orchestra orchestra = new Orchestra();
  @Log.NT private static final ArrayList<TalonFX> talons = new ArrayList<>(11);
  private static List<String> fileNames =
      List.of(
          "baka.chrp",
          "blue.chrp",
          "BWomp.chrp",
          "gas.chrp",
          "hopes-and-dreams.chrp",
          "last-surprise.chrp",
          "magical-toy-box.chrp",
          "meg.chrp",
          "rick.chrp",
          "running.chrp",
          "slider.chrp",
          "spidAAAAAA.chrp",
          "TZK.chrp",
          "floor-of-lava.chrp",
          "prelude-in-g-minor.chrp");

  private static boolean fileLoaded = false;

  private static SendableChooser<Runnable> songChooser = new SendableChooser<>();

  static {
    for (String fileName : fileNames) {
      songChooser.addOption(fileName, () -> TalonUtils.loadOrchestraFile(fileName));
    }

    SmartDashboard.putData("Queued Song", songChooser);
    songChooser.onChange(chooseSelected -> chooseSelected.run());
  }

  /**
   * Adds motor to the orchestra.
   *
   * @param talon The motor to add.
   */
  public static void addMotor(TalonFX talon) {
    talons.add(talon);
  }

  public static Command getSelected() {
    return Commands.runOnce(songChooser.getSelected());
  }

  /**
   * Add all motors to the Orchestra. Should be called once after addition of all Talons to
   * TalonUtils.
   *
   * <p>Use {@link#loadOrchestraFile()} to change the played file.
   *
   * @return Whether loading the file was successful.
   */
  public static void configureOrchestra() {
    AudioConfigs audioCfg = new AudioConfigs().withAllowMusicDurDisable(true);
    for (TalonFX talon : talons) {
      talon.getConfigurator().apply(audioCfg);
      orchestra.addInstrument(talon);
    }
  }

  /**
   * Load the selected CHRP file located in the deploy directory.
   *
   * @param fileName The name of the file to play.
   * @return Whether loading the file was successful.
   */
  public static boolean loadOrchestraFile(String fileName) {
    fileLoaded = orchestra.loadMusic(fileName).isOK();
    if (!fileLoaded) fileNotFound();
    return fileLoaded;
  }

  /**
   * Begin playback of the loaded file.
   *
   * @return Whether the operation was successful.
   */
  public static boolean play() {
    if (fileLoaded) {
      return orchestra.play().isOK();
    }
    fileNotFound();
    return false;
  }

  /**
   * Stop and restart playback of the loaded file.
   *
   * @return Whether the operation was successful.
   */
  public static boolean stop() {
    if (fileLoaded) {
      return orchestra.stop().isOK();
    }
    fileNotFound();
    return false;
  }

  /**
   * Pause playback of the loaded file.
   *
   * @return Whether the operation was successful.
   */
  public static boolean pause() {
    if (fileLoaded) {
      return orchestra.pause().isOK();
    }
    fileNotFound();
    return false;
  }

  private static void fileNotFound() {
    fileLoaded = false;
    FaultLogger.report(
        "Orchestra",
        "CHRP file not loaded. Check that it is in the deploy directory & includes file extension.",
        FaultType.WARNING);
  }
}
