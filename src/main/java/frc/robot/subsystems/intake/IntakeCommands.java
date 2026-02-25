package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.intake.IntakeConstants.PIVOT_CONSTANTS;
import static frc.robot.subsystems.intake.IntakeConstants.ROLLER_CONSTANTS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.logging.TunableDouble;
import frc.robot.util.states.premade.RadVel_State;
import java.util.function.DoubleSupplier;

public class IntakeCommands {
  private static final DoubleSupplier targetSpeed_radPs =
      new TunableDouble(ROLLER_CONSTANTS.tuningLogName + "/Targets/_radPs", 80, () -> true);
  private static final DoubleSupplier lowered_rad = new TunableDouble(
      PIVOT_CONSTANTS.tuningLogName + "/Targets/lowered_rad",
      (PIVOT_CONSTANTS.min_Pos.pos(Radians) + PIVOT_CONSTANTS.max_Pos.pos(Radians)) / 2,
      () -> true);

  public static Command lowerCommand(IntakeSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setPivotGoal(PIVOT_CONSTANTS.min_Pos);
          // runRollerWhenLowered(instance);
          // instance.setRollerGoal(new RadVel_State(0));
        },
        instance);
  }

  public static Command raiseCommand(IntakeSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setPivotGoal(PIVOT_CONSTANTS.max_Pos);
          // runRollerWhenLowered(instance);
          // instance.setRollerGoal(new RadVel_State(targetSpeed_radPs.getAsDouble()));
        },
        instance);
  }

  private static void runRollerWhenLowered(IntakeSubsystem instance) {
    if (instance.getPivotState().pos(Radians) < lowered_rad.getAsDouble()) {
      instance.setRollerGoal(new RadVel_State(targetSpeed_radPs.getAsDouble()));
    } else {
      instance.setRollerGoal(new RadVel_State(0));
    }
  }
  // for tuning
  public static Command RunRollerForwardForTuning(IntakeSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setRollerGoal(new RadVel_State(targetSpeed_radPs.getAsDouble()));
        },
        instance);
  }

  public static Command RunRollerBackwardForTuning(IntakeSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setRollerGoal(new RadVel_State(-targetSpeed_radPs.getAsDouble()));
        },
        instance);
  }

  public static Command stopCommand(IntakeSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setRollerGoal(new RadVel_State(0));
        },
        instance);
  }
}
