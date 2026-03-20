package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.LOG_NAME;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.logging.TunableDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IndexerCommands {
  private static final String TUNING_NT_KEY = "Tuning/" + LOG_NAME + "/Commands";
  private static final DoubleSupplier forwardSpeed =
      new TunableDouble(TUNING_NT_KEY + "/ForwardSpeed", 1);
  private static final DoubleSupplier reverseSpeed =
      new TunableDouble(TUNING_NT_KEY + "/ReverseSpeed", -0.5);

  public static Command stop(IndexerSubsystem indexer) {
    return indexer.run(() -> {
      indexer.setHopperSpeed(0);
      indexer.setKickerSpeed(0);
    });
  }

  private static boolean conditionalRunForward_isRunning = false;

  public static Command conditionalRunForward(IndexerSubsystem indexer, BooleanSupplier shouldRun) {
    Timer timer = new Timer();
    timer.start();
    return indexer.startRun(
        () -> {
          conditionalRunForward_isRunning = false;
        },
        () -> {
          Logger.recordOutput(TUNING_NT_KEY + "/indexertimer", timer.get());
          Logger.recordOutput(TUNING_NT_KEY + "/indexerrunning", conditionalRunForward_isRunning);
          if (shouldRun.getAsBoolean()) {
            if (!conditionalRunForward_isRunning) {
              timer.reset();
              conditionalRunForward_isRunning = true;
            }
            if (timer.hasElapsed(0.15)) {
              indexer.setHopperSpeed(forwardSpeed.getAsDouble());
              indexer.setKickerSpeed(forwardSpeed.getAsDouble());
              if (timer.hasElapsed(2.15)) {
                timer.reset();
              }
            } else {
              indexer.setHopperSpeed(reverseSpeed.getAsDouble());
              indexer.setKickerSpeed(reverseSpeed.getAsDouble());
            }
          } else {
            conditionalRunForward_isRunning = false;
            indexer.setHopperSpeed(0);
            indexer.setKickerSpeed(0);
          }
        });
  }

  public static Command runForwardCommand(IndexerSubsystem indexer) {
    // return indexer.run(() -> {
    //   indexer.setHopperSpeed(forwardSpeed.getAsDouble());
    //   indexer.setKickerSpeed(forwardSpeed.getAsDouble());
    // });
    Timer timer = new Timer();
    timer.start();
    return indexer.startRun(
        () -> {
          timer.reset();
        },
        () -> {
          if (timer.hasElapsed(0.15)) {
            indexer.setHopperSpeed(forwardSpeed.getAsDouble());
            indexer.setKickerSpeed(forwardSpeed.getAsDouble());
            if (timer.hasElapsed(2.15)) {
              timer.reset();
            }
          } else {
            indexer.setHopperSpeed(reverseSpeed.getAsDouble());
            indexer.setKickerSpeed(reverseSpeed.getAsDouble());
          }
        });
  }

  public static Command runBackwardCommand(IndexerSubsystem indexer) {
    return indexer.run(() -> {
      indexer.setHopperSpeed(reverseSpeed.getAsDouble());
      indexer.setKickerSpeed(reverseSpeed.getAsDouble());
    });
  }
}
