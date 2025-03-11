package org.sciborgs1155.lib;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Hydromands {
    public static Command sequence(Command... commands) {
        return Commands.sequence(proxyAll(commands));
    }
    
    public static Command parallel(Command... commands) {
        return Commands.parallel(proxyAll(commands));
    }

    public static Command deadline(Command deadline, Command... otherCommands) {
        return Commands.deadline(deadline.asProxy(), proxyAll(otherCommands));
    }

    public static Command[] proxyAll(Command... commands) {
        return Arrays.stream(commands).map(command -> command.asProxy()).toArray(Command[]::new);
    }
}
