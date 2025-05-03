package org.sciborgs1155.lib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Arrays;

public class ProxyCommandGroups {
  /**
   * Runs a group of commands, one after another. All commands in the group are proxied and the
   * overall command group therefore requires no subsystems.
   */
  public static Command sequence(Command... commands) {
    return Commands.sequence(proxyAll(commands));
  }

  /**
   * Runs a group of commands at the same time. Ends once all commands in the group finish. All
   * commands in the group are proxied and the overall command group therefore requires no
   * subsystems.
   */
  public static Command parallel(Command... commands) {
    return Commands.parallel(proxyAll(commands));
  }

  /**
   * Runs a group of commands at the same time. Ends once a specific command finishes, and cancels
   * the others. All commands in the group are proxied and the overall command group therefore
   * requires no subsystems.
   *
   * @param deadline the command which cancels all other commands once it ends or is canceled.
   * @param otherCommands all other commands run in parallel.
   */
  public static Command deadline(Command deadline, Command... otherCommands) {
    return Commands.deadline(deadline.asProxy(), proxyAll(otherCommands));
  }

  /** proxies all commands and returns them as a list */
  public static Command[] proxyAll(Command... commands) {
    return Arrays.stream(commands).map(command -> command.asProxy()).toArray(Command[]::new);
  }
}
