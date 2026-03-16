package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.NT_KEY;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeConstants.Pivot;
import frc.robot.util.logging.TunableBoolean;
import frc.robot.util.logging.TunableDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class IntakeCommands {
  private static final String TUNING_NT_KEY = "Tuning/" + NT_KEY + "/Commands";
  private static final BooleanSupplier PIVOT_WORKS =
      new TunableBoolean(TUNING_NT_KEY + "/PivotWorks", true);
  private static final BooleanSupplier PIVOT_KINDA_WORKS =
      new TunableBoolean(TUNING_NT_KEY + "/PivotKindaWorks", true);
  private static final DoubleSupplier PIVOT_LOWER_THRESHOLD =
      new TunableDouble(TUNING_NT_KEY + "/PivotLowerThreshold_rad", -1.4);
  private static final DoubleSupplier ROLLER_THRESHOLD =
      new TunableDouble(TUNING_NT_KEY + "/RollerThreshold_radPs", 10);
  private static final double ROLLER_FORWARD_SPEED = 1.0;
  private static final DoubleSupplier ROLLER_REVERSE_SPEED =
      new TunableDouble(TUNING_NT_KEY + "/RollerReverseSpeed", -0.5);
  // private static final DoubleSupplier lowered_rad = new TunableDouble(
  //     TUNING_NT_KEY + "/lowered_rad",
  //     (PIVOT_CONSTANTS.min_Pos.pos(Radians) + PIVOT_CONSTANTS.max_Pos.pos(Radians)) / 2,
  //     () -> true);
  private static final DoubleSupplier up_pos_supplier =
      new TunableDouble(TUNING_NT_KEY + "/up_pos", -0.67);
  private static final Supplier<Angle> SHOOTING_POS =
      () -> Radians.of(up_pos_supplier.getAsDouble());
  private static final Angle DOWN_POS = Pivot.MIN_POS;
  public static boolean shooting = false;

  // public static void setIndexing(boolean isIndexing) {
  //   shooting = isIndexing;
  // }

  // public static Command setIndexingCommand(BooleanSupplier isIndexing) {
  //   return Commands.runOnce(() -> shooting = isIndexing.getAsBoolean());
  // }

  // public static enum IntakeState {
  //   RAISED_IDLE,
  //   LOWERED_IDLE,
  //   INTAKING,
  //   OUTTAKING,
  //   INDEXING
  // }

  public static Command defaultCommand(IntakeSubsystem intake, BooleanSupplier raisePivot) {
    return Commands.run(
        () -> {
          if (shooting) {
            intake.setRollerSpeed(ROLLER_FORWARD_SPEED);
            if (PIVOT_WORKS.getAsBoolean()) {
              intake.setPivotGoalPos(SHOOTING_POS.get());
            } else {
              if (PIVOT_KINDA_WORKS.getAsBoolean()
                  && intake.getPivotPos().in(Radians) > PIVOT_LOWER_THRESHOLD.getAsDouble()) {
                intake.setPivotGoalPos(DOWN_POS);
              } else {
                intake.setPivotVoltage(Volts.zero());
              }
            }
          } else {
            intake.setRollerSpeed(0);
            if (PIVOT_WORKS.getAsBoolean()) {
              if (raisePivot.getAsBoolean()) {
                intake.setPivotGoalPos(SHOOTING_POS.get());
              } else {
                intake.setPivotGoalPos(DOWN_POS);
              }
            } else {
              if (PIVOT_KINDA_WORKS.getAsBoolean()
                  && intake.getPivotPos().in(Radians) > PIVOT_LOWER_THRESHOLD.getAsDouble()) {
                intake.setPivotGoalPos(DOWN_POS);
              } else {
                intake.setPivotVoltage(Volts.zero());
              }
            }
          }
        },
        intake);
  }

  public static Command intakeFuel(IntakeSubsystem intake, BooleanSupplier raisePivot) {
    return Commands.run(
        () -> {
          intake.setRollerSpeed(ROLLER_FORWARD_SPEED);
          if (PIVOT_WORKS.getAsBoolean()) {
            if (raisePivot.getAsBoolean()
                || intake.getRollerVel().in(RadiansPerSecond) < ROLLER_THRESHOLD.getAsDouble()) {
              intake.setPivotGoalPos(SHOOTING_POS.get());
            } else {
              intake.setPivotGoalPos(DOWN_POS);
            }
          } else {
            if (PIVOT_KINDA_WORKS.getAsBoolean()
                && intake.getPivotPos().in(Radians) > PIVOT_LOWER_THRESHOLD.getAsDouble()) {
              intake.setPivotGoalPos(DOWN_POS);
            } else {
              intake.setPivotVoltage(Volts.zero());
            }
          }
        },
        intake);
  }

  public static Command outtakeFuel(IntakeSubsystem intake, BooleanSupplier raisePivot) {
    return Commands.run(
        () -> {
          intake.setRollerSpeed(ROLLER_REVERSE_SPEED.getAsDouble());
          if (PIVOT_WORKS.getAsBoolean()) {
            if (raisePivot.getAsBoolean()) {
              intake.setPivotGoalPos(SHOOTING_POS.get());
            } else {
              intake.setPivotGoalPos(DOWN_POS);
            }
          } else {
            if (PIVOT_KINDA_WORKS.getAsBoolean()
                && intake.getPivotPos().in(Radians) > PIVOT_LOWER_THRESHOLD.getAsDouble()) {
              intake.setPivotGoalPos(DOWN_POS);
            } else {
              intake.setPivotVoltage(Volts.zero());
            }
          }
        },
        intake);
  }

  /** Lowers the pivot without running the roller. Ends when pivot is within a set error of its down position. */
  public static Command lowerIntake(IntakeSubsystem intake) {
    return Commands.run(
            () -> {
              intake.setPivotGoalPos(DOWN_POS);
            },
            intake)
        .onlyWhile(() -> intake.getPivotPos().gt(DOWN_POS.plus(Radians.of(0.04))));
  }

  public static Command lowerCommand(IntakeSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setPivotGoalPos(Radians.of(-1.5));
          // runRollerWhenLowered(instance);
          // instance.setRollerGoal(new RadVel_State(0));
        },
        instance);
  }

  public static Command midAndRunCommand(IntakeSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setPivotGoalPos(Radians.of(-0.7));
          instance.setRollerSpeed(1);
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
          instance.setPivotGoalPos(Pivot.MIN_POS);
          instance.setRollerSpeed(ROLLER_FORWARD_SPEED);
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
          instance.setRollerSpeed(ROLLER_FORWARD_SPEED);
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
