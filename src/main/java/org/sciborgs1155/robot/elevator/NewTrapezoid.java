package org.sciborgs1155.robot.elevator;

/**
 * Note that the very large majority of the logic here is entirely based off of wpilib's Trapezoid
 * profile.
 *
 * <p>Modified Trapezoid Profile which uses varying acceleration in the upwards direction vs. the
 * downwards acceleration.
 */
public class NewTrapezoid {
  private double direction = 1;
  private final Constraints constraints;

  private State goal = new State();
  private State current = new State();

  public NewTrapezoid(Constraints constraints) {
    this.constraints = constraints;
  }

  public State goal() {
    return goal;
  }

  public State current() {
    return current;
  }

  public boolean atGoal() {
    return current.equals(goal);
  }

  /** If you want to use zero velocity setpoints */ 
  public State calculate(double t, double current, double goal) {
    return calculate(t, new State(current, 0), new State(goal, 0));
  }

  public State calculate(double t, State current, State goal) {
    this.goal = goal;

    double startAccel =
        shouldFlipAcceleration(current, goal)
            ? constraints.downwardsAccel
            : constraints.upwardsAccel;
    double endAccel =
        shouldFlipAcceleration(current, goal)
            ? constraints.upwardsAccel
            : constraints.downwardsAccel;

    direction = shouldFlipAcceleration(current, goal) ? -1 : 1;
    current = direct(current);
    goal = direct(goal);

    if (current.velocity > constraints.maxVelocity) {
      current.velocity = constraints.maxVelocity;
    }

    // Deal with a possibly truncated motion profile (with nonzero initial or
    // final velocity) by calculating the parameters as if the profile began and
    // ended at zero velocity
    double cutoffBegin = current.velocity / startAccel;
    double cutoffDistBegin = cutoffBegin * cutoffBegin * startAccel / 2.0;

    double cutoffEnd = goal.velocity / endAccel;
    double cutoffDistEnd = cutoffEnd * cutoffEnd * endAccel / 2.0;

    // Now we can calculate the parameters as if it was a full trapezoid instead
    // of a truncated one
    double fullTrapezoidDist = cutoffDistBegin + (goal.position - current.position) + cutoffDistEnd;

    double upwardsAccelerationTime = constraints.maxVelocity / constraints.upwardsAccel;
    double downwardsAccelerationTime = constraints.maxVelocity / constraints.downwardsAccel;

    double fullSpeedDist =
        fullTrapezoidDist
            - (upwardsAccelerationTime * upwardsAccelerationTime * constraints.upwardsAccel
                    + downwardsAccelerationTime
                        * downwardsAccelerationTime
                        * constraints.downwardsAccel)
                / 2;

    // Handle the case where the profile never reaches full speed
    //https://www.desmos.com/calculator/9otnakisu0
    if (fullSpeedDist < 0) {
      upwardsAccelerationTime = Math.sqrt((2 * fullTrapezoidDist) / (constraints.upwardsAccel + (constraints.upwardsAccel / constraints.downwardsAccel) + 1));
      downwardsAccelerationTime = constraints.upwardsAccel * upwardsAccelerationTime / constraints.downwardsAccel;
      fullSpeedDist = 0;
    }

    double endAccelTime = upwardsAccelerationTime - cutoffBegin;
    double endFullSpeed = endAccelTime + fullSpeedDist / constraints.maxVelocity;
    double endDecelTime = endFullSpeed + downwardsAccelerationTime - cutoffEnd;

    State result = new State(current.position, current.velocity);

    if (t < endAccelTime) {
      result.velocity += t * constraints.upwardsAccel;
      result.position += (current.velocity + t * constraints.upwardsAccel / 2.0) * t;
    } else if (t < endFullSpeed) {
      result.velocity = constraints.maxVelocity;
      result.position +=
          (current.velocity + endAccelTime * constraints.upwardsAccel / 2.0) * endAccelTime
              + constraints.maxVelocity * (t - endAccelTime);
    } else if (t <= endDecelTime) {
      result.velocity = goal.velocity + (endDecelTime - t) * constraints.downwardsAccel;
      double timeLeft = endDecelTime - t;
      result.position =
          goal.position - (goal.velocity + timeLeft * constraints.downwardsAccel / 2.0) * timeLeft;
    } else {
      result = goal;
    }
    this.current = direct(result);
    return direct(result);
  }

  private static boolean shouldFlipAcceleration(State initial, State goal) {
    return initial.position > goal.position;
  }

  // Flip the sign of the velocity and position if the profile is inverted
  private State direct(State in) {
    State result = new State(in.position, in.velocity);
    result.position = result.position * direction;
    result.velocity = result.velocity * direction;
    return result;
  }

  /** Profile state. */
  public static class State {
    /** The position at this state. */
    public double position;

    /** The velocity at this state. */
    public double velocity;

    /** Default constructor. */
    public State() {}

    /**
     * Constructs constraints for a Trapezoid Profile.
     *
     * @param position The position at this state.
     * @param velocity The velocity at this state.
     */
    public State(double position, double velocity) {
      this.position = position;
      this.velocity = velocity;
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof State rhs
          && rhs.position == this.position
          && rhs.velocity == this.velocity;
    }
  }

  /** Profile constraints. */
  public static class Constraints {
    /** Maximum velocity. */
    public final double maxVelocity;

    /** Maximum upwards acceleration. */
    public final double upwardsAccel;

    /** Maximum downards acceleration */
    public final double downwardsAccel;

    /**
     * Constructs constraints for a TrapezoidProfile.
     *
     * @param maxVelocity maximum velocity
     * @param maxAcceleration maximum acceleration
     */
    public Constraints(double maxVelocity, double upwardsAccel, double downwardsAccel) {
      this.maxVelocity = maxVelocity;
      this.upwardsAccel = upwardsAccel;
      this.downwardsAccel = downwardsAccel;
    }
  }
}
