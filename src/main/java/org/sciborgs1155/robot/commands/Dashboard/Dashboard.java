package org.sciborgs1155.robot.commands.Dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;

public class Dashboard implements Logged {
  private static final Display display = new Display();
  private static String selectedBranch = "";
  private static String selectedLevel = "";
  private static boolean processor = false;

  public static void configure() {
    display.branches.forEach(
        button -> button.pressed.onTrue(Commands.runOnce(() -> selectedBranch = button.name)));
    display.levels.forEach(
        button -> button.pressed.onTrue(Commands.runOnce(() -> selectedLevel = button.name)));
    reset()
        .onTrue(
            Commands.runOnce(
                () -> {
                  selectedBranch = "";
                  selectedLevel = "";
                  processor = false;
                }));
    Trigger processorTrigger = new Trigger(() -> display.processor.getModel().isPressed());
    processorTrigger.onTrue(Commands.runOnce(() -> processor = !processor));
    processorTrigger.onTrue(
        Commands.runOnce(
            () -> {
              selectedBranch = "";
              selectedLevel = "";
            }));
  }

  public static Trigger go() {
    return new Trigger(() -> display.GO.getModel().isPressed());
  }

  public static Trigger reset() {
    return new Trigger(() -> display.RESET.getModel().isPressed());
  }

  public static Trigger processor() {
    return new Trigger(() -> processor).and(go());
  }

  public static void update() {
    SmartDashboard.putBoolean("processor", processor);
    SmartDashboard.putString("selectedBranch", selectedBranch);
    SmartDashboard.putString("selectedLevel", selectedLevel);
  }

  public static enum Branch {
    A("A"),
    B("B"),
    C("C"),
    D("D"),
    E("E"),
    F("F"),
    G("G"),
    H("H"),
    I("I"),
    J("J"),
    K("K"),
    L("L");

    public Trigger trigger;

    private Branch(String name) {
      this.trigger = new Trigger(() -> selectedBranch == name).and(go());
    }
  }

  public static enum Level {
    L1("1"),
    L2("2"),
    L3("3"),
    L4("4");

    public Trigger trigger;

    private Level(String name) {
      this.trigger = new Trigger(() -> selectedLevel == name).and(go());
    }
  }
}
