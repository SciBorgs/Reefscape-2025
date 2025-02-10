package org.sciborgs1155.robot.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * A class for faster Talon Odometry using a faster thread.
 *
 * <p>Inspired by 6328's PhoenixOdometryThread.
 */
public class TalonOdometryThread extends Thread {
  private BaseStatusSignal[] talonSignals = new BaseStatusSignal[0];
  private final List<Queue<Double>> talonQueues = new ArrayList<>();
  private final List<DoubleSupplier> otherSignals = new ArrayList<>();
  private final List<Queue<Double>> otherQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static TalonOdometryThread instance = null;

  public static TalonOdometryThread getInstance() {
    if (instance == null) {
      instance = new TalonOdometryThread();
    }
    return instance;
  }

  @Override
  public synchronized void start() {
    super.start();
  }

  public Queue<Double> registerSignal(StatusSignal<Double> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    // Drive.lock.writeLock().lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[talonSignals.length + 1];
      System.arraycopy(talonSignals, 0, newSignals, 0, talonSignals.length);
      newSignals[talonSignals.length] = signal;
      talonSignals = newSignals;
      talonQueues.add(queue);
    } finally {
      // Drive.lock.writeLock().unlock();
    }
    return queue;
  }

  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    // Drive.lock.writeLock().lock();
    try {
      otherSignals.add(signal);
      otherQueues.add(queue);
    } finally {
      // Drive.lock.writeLock().unlock();
    }
    return queue;
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    // Drive.lock.writeLock().lock();
    try {
      timestampQueues.add(queue);
    } finally {
      // Drive.lock.writeLock().lock();
    }
    return queue;
  }

  @Override
  public void run() {
    // TODO write
  }
}
