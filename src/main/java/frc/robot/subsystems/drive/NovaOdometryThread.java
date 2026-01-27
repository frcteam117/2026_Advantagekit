// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version includes an overload for Spark signals, which checks for errors to ensure that
 * all measurements in the sample are valid.
 */
public class NovaOdometryThread {
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static NovaOdometryThread instance = null;
  private Notifier notifier = new Notifier(this::run);

  public static NovaOdometryThread getInstance() {
    if (instance == null) {
      instance = new NovaOdometryThread();
    }
    return instance;
  }

  private NovaOdometryThread() {
    notifier.setName("OdometryThread");
  }

  public void start() {
    if (!timestampQueues.isEmpty()) {
      notifier.startPeriodic(1.0 / DriveConstants.odometryFrequency_Hz);
    }
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  private void run() {
    // Save new data to queues
    Drive.odometryLock.lock();
    try {
      // Get sample timestamp
      double timestamp = RobotController.getFPGATime() / 1e6;

      // Read and add values to queues
      // ThriftyNovas only have configure errors, so we cannot check validity in the same way as
      // sparks
      for (int i = 0; i < genericSignals.size(); i++) {
        genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
      }
      for (int i = 0; i < timestampQueues.size(); i++) {
        timestampQueues.get(i).offer(timestamp);
      }
    } finally {
      Drive.odometryLock.unlock();
    }
  }
}
