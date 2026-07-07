package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.LOG_NAME;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.logging.TunableDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IndexerCommands {
  private static final String TUNING_NT_KEY = "Tuning/" + LOG_NAME + "/Commands";
  private static final DoubleSupplier forwardSpeed =
      new TunableDouble(TUNING_NT_KEY + "/ForwardSpeed", 1);
  private static final DoubleSupplier reverseSpeed =
      new TunableDouble(TUNING_NT_KEY + "/ReverseSpeed", -0.9);
  public static boolean runningBackwards = false;

  public static Command stop(IndexerSubsystem indexer) {
    return indexer.run(() -> {
      indexer.setHopperSpeed(0);
      indexer.setKickerSpeed(0);
    });
  }

  public static Command intakingAgitation(IndexerSubsystem indexer) {
    return indexer.run(() -> {
      indexer.setHopperSpeed(0.1);
      indexer.setKickerSpeed(-0.15);
    });
  }

  public static Command conditionalRunForward(IndexerSubsystem indexer, BooleanSupplier shouldRun) {
    return indexer
        .run(() -> {
          indexer.setHopperSpeed(forwardSpeed.getAsDouble());
          indexer.setKickerSpeed(forwardSpeed.getAsDouble());
          indexer.isPreloading = true;
        })
        .withTimeout(.2)
        .andThen(indexer.run(() -> {
          if (shouldRun.getAsBoolean()) {
            indexer.isPreloading = false;
            if (runningBackwards) {
              indexer.setHopperSpeed(reverseSpeed.getAsDouble());
              indexer.setKickerSpeed(reverseSpeed.getAsDouble());
            } else {
              indexer.setHopperSpeed(forwardSpeed.getAsDouble());
              indexer.setKickerSpeed(forwardSpeed.getAsDouble());
            }
          } else {
            indexer.setHopperSpeed(0);
            indexer.setKickerSpeed(0);
          }
        }))
        .withName("Intake_conditionalRunForward");
  }

  public static Command runForwardCommand(IndexerSubsystem indexer) {
    return indexer.run(() -> {
      if (runningBackwards) {
        indexer.setHopperSpeed(reverseSpeed.getAsDouble());
        indexer.setKickerSpeed(reverseSpeed.getAsDouble());
      } else {
        indexer.setHopperSpeed(forwardSpeed.getAsDouble());
        indexer.setKickerSpeed(forwardSpeed.getAsDouble());
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
