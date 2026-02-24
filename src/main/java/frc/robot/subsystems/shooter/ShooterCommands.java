package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.logging.TunableDouble;
import frc.robot.util.states.premade.RadVel_State;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
  private static final DoubleSupplier targetSpeed_radPs =
      new TunableDouble(FLYWHEEL_CONSTANTS.tuningLogName + "/_radPs", 240, () -> true);

  public static Command stopCommand(ShooterSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setFlywheelGoal(new RadVel_State(0));
        },
        instance);
  }

  public static Command runCommand(ShooterSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setFlywheelGoal(new RadVel_State(targetSpeed_radPs.getAsDouble()));
        },
        instance);
  }

  public static Command hoodSysId(ShooterSubsystem shooter, SysIdType type) {
    return Commands.run(() -> {}, shooter)
        .withTimeout(1)
        .andThen(shooter.getHoodSysIdCommand(type))
        .withName("HoodSysId_" + type.name());
  }

  public static Command flywheelSysId(ShooterSubsystem shooter, SysIdType type) {
    return Commands.run(() -> {}, shooter)
        .withTimeout(1)
        .andThen(shooter.getFlywheelSysIdCommand(type))
        .withName("FlywheelSysId_" + type.name());
  }

  // public static Command runFlywheelGoalVelocity(Shooter shooter, DoubleSupplier supplier_radPs) {
  //   return Commands.run(
  //       () -> shooter.setFlywheelGoalVelocity(supplier_radPs.getAsDouble()), shooter);
  // }

  // public static Command runHoodGoalState(
  //     Shooter shooter, DoubleSupplier supplier_rad, DoubleSupplier supplier_radPs) {
  //   return Commands.run(
  //       () -> shooter.setHoodGoalState(supplier_rad.getAsDouble(), supplier_radPs.getAsDouble()),
  //       shooter);
  // }

  // public static Command runHoodGoalPosition(Shooter shooter, DoubleSupplier supplier_rad) {
  //   return runHoodGoalState(shooter, supplier_rad, () -> 0);
  // }

  // public static Command trackPosition(
  //     Shooter shooter, Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d>
  // targetSupplier) {
  //   return Commands.run(
  //       () -> {
  //         Translation2d robotToTarget =
  //             targetSupplier.get().minus(robotPoseSupplier.get().getTranslation());
  //         shooter.setTurretGoalPosition(robotToTarget.getAngle().getRadians()
  //             - robotPoseSupplier.get().getRotation().getRadians());
  //         double pitch_rad = Math.atan(UnitUtil.ftTom(16) / robotToTarget.getNorm());
  //         shooter.setHoodGoalPosition(pitch_rad);
  //         shooter.setFlywheelGoalVelocity(
  //             UnitUtil.ftTom(16) / Math.sin(pitch_rad) / UnitUtil.inTom(2));
  //       },
  //       shooter);
  // }
}
