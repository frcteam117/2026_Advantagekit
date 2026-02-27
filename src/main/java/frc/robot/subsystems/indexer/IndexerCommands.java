package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.logging.TunableDouble;
import frc.robot.util.states.premade.RadVel_State;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IndexerCommands {
  private static final DoubleSupplier forward_radPs = new TunableDouble(
      "Tuning/" + IndexerConstants.LOG_NAME + "/Targets/forward_radPs", 30, () -> true);
  private static final DoubleSupplier backward_radPs = new TunableDouble(
      "Tuning/" + IndexerConstants.LOG_NAME + "/Targets/backward_radPs", -10, () -> true);

  public static Command stop(IndexerSubsystem indexer) {
    return indexer.run(() -> {
      indexer.setHopperGoal(new RadVel_State(0));
      indexer.setKickerGoal(new RadVel_State(0));
    });
  }

  public static Command conditionalRunForward(IndexerSubsystem indexer, BooleanSupplier shouldRun) {
    return indexer.run(() -> {
      if (shouldRun.getAsBoolean()) {
        indexer.setHopperGoal(new RadVel_State(forward_radPs.getAsDouble()));
        indexer.setKickerGoal(new RadVel_State(forward_radPs.getAsDouble()));
      } else {
        indexer.setHopperGoal(new RadVel_State(0));
        indexer.setKickerGoal(new RadVel_State(0));
      }
    });
  }

  public static Command runForwardCommand(IndexerSubsystem indexer) {
    return indexer.run(() -> {
      indexer.setHopperGoal(new RadVel_State(forward_radPs.getAsDouble()));
      indexer.setKickerGoal(new RadVel_State(forward_radPs.getAsDouble()));
    });
  }

  public static Command runBackwardCommand(IndexerSubsystem indexer) {
    return indexer.run(() -> {
      indexer.setHopperGoal(new RadVel_State(backward_radPs.getAsDouble()));
      indexer.setKickerGoal(new RadVel_State(backward_radPs.getAsDouble()));
    });
  }
}
