package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.intake.IntakeConstants.NT_KEY;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.logging.TunableDouble;
import java.util.function.DoubleSupplier;

public class IntakeCommands {
  private static final String TUNING_NT_KEY = "Tuning/" + NT_KEY + "/Commands";
  private static final DoubleSupplier ROLLER_FORWARD_SPEED =
      new TunableDouble(TUNING_NT_KEY + "/RollerForwardSpeed", 1, () -> true);
  private static final DoubleSupplier ROLLER_REVERSE_SPEED =
      new TunableDouble(TUNING_NT_KEY + "/RollerReverseSpeed", -0.5, () -> true);
  // private static final DoubleSupplier lowered_rad = new TunableDouble(
  //     TUNING_NT_KEY + "/lowered_rad",
  //     (PIVOT_CONSTANTS.min_Pos.pos(Radians) + PIVOT_CONSTANTS.max_Pos.pos(Radians)) / 2,
  //     () -> true);

  public static Command lowerCommand(IntakeSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setPivotGoalPos(Radians.of(-1.5));
          // runRollerWhenLowered(instance);
          // instance.setRollerGoal(new RadVel_State(0));
        },
        instance);
  }

  public static Command midCommand(IntakeSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setPivotGoalPos(Radians.of(-0.7));
          // runRollerWhenLowered(instance);
          // instance.setRollerGoal(new RadVel_State(0));
        },
        instance);
  }

  public static Command raiseCommand(IntakeSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setPivotGoalPos(Radians.of(0));
          // runRollerWhenLowered(instance);
          // instance.setRollerGoal(new RadVel_State(targetSpeed_radPs.getAsDouble()));
        },
        instance);
  }

  public static Command runRollerAndLowerPivot(IntakeSubsystem instance) {
    return Commands.run(
        () -> {
          // instance.setPivotGoal(PIVOT_CONSTANTS.min_Pos);
          instance.setRollerSpeed(ROLLER_FORWARD_SPEED.getAsDouble());
          // runRollerWhenLowered(instance);
          // instance.setRollerGoal(new RadVel_State(targetSpeed_radPs.getAsDouble()));
        },
        instance);
  }

  // private static void runRollerWhenLowered(IntakeSubsystem instance) {
  //   if (instance.getPivotState().pos(Radians) < lowered_rad.getAsDouble()) {
  //     instance.setRollerGoal(new RadVel_State(targetSpeed_radPs.getAsDouble()));
  //   } else {
  //     instance.setRollerGoal(new RadVel_State(0));
  //   }
  // }
  // for tuning
  public static Command runRollerForward(IntakeSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setRollerSpeed(ROLLER_FORWARD_SPEED.getAsDouble());
        },
        instance);
  }

  public static Command runRollerBackward(IntakeSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setRollerSpeed(ROLLER_REVERSE_SPEED.getAsDouble());
        },
        instance);
  }

  public static Command stopCommand(IntakeSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setRollerSpeed(0);
          // instance.setPivotGoal(new RadVel_State(0));
        },
        instance);
  }
}
