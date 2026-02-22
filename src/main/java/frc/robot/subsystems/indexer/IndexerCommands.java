package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.logging.TunableDouble;
import frc.robot.util.states.premade.RadVel_State;
import java.util.function.DoubleSupplier;

public class IndexerCommands {
  private static final DoubleSupplier forward_radPs = new TunableDouble(
      "Tuning/" + IndexerConstants.LOG_NAME + "/Targets/forward_radPs", 40, () -> true);
  private static final DoubleSupplier backward_radPs = new TunableDouble(
      "Tuning/" + IndexerConstants.LOG_NAME + "/Targets/backward_radPs", -15, () -> true);

  public static Command stopCommand(IndexerSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setHopperGoal(new RadVel_State(0));
          instance.setKickerGoal(new RadVel_State(0));
        },
        instance);
  }

  public static Command runForwardCommand(IndexerSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setHopperGoal(new RadVel_State(forward_radPs.getAsDouble()));
          instance.setKickerGoal(new RadVel_State(forward_radPs.getAsDouble()));
        },
        instance);
  }

  public static Command runBackwardCommand(IndexerSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setHopperGoal(new RadVel_State(backward_radPs.getAsDouble()));
          instance.setKickerGoal(new RadVel_State(backward_radPs.getAsDouble()));
        },
        instance);
  }
}
