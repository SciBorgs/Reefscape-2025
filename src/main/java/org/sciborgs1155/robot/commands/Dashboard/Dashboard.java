package org.sciborgs1155.robot.commands.Dashboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;
import monologue.Annotations.Log;

import org.sciborgs1155.robot.commands.Dashboard.Display.Button;

import edu.wpi.first.wpilibj2.command.Commands;

public class Dashboard implements Logged {
    private final Display display = new Display();
    @Log.NT private boolean go = false;

    public Dashboard() {
        L1().onTrue(Commands.run(() -> System.out.println("test")));

        // while (true) {
        //  }
    }

    public Trigger L1() {
        return new Trigger(() -> display.L1.getModel().isPressed());
    }

    public Trigger go() {
        return new Trigger(() -> go);
    }

    public void update() {
        if (display.L1.getModel().isPressed()) {
            go = true;
        }
    }
}
