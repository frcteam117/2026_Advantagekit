package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.logging.TunableDouble;
import frc.robot.util.states.premade.RadVel_State;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterCommands {
  private static final String TUNING_NT_KEY = "Tuning/" + LOG_NAME + "/Commands";
  private static final DoubleSupplier targetSpeed_radPs =
      new TunableDouble(TUNING_NT_KEY + "/Flywheel_radPs", 471, () -> true);
  private static final DoubleSupplier targetHoodSpeed_radPs =
      new TunableDouble(TUNING_NT_KEY + "/Hood_radPs", .2, () -> true);
  // private static final InterpolatingDoubleTreeMap distanceToSpeedLerp =
  //     new InterpolatingDoubleTreeMap();

  // static {
  //   distanceToSpeedLerp.put(0.0, 0.0);
  //   distanceToSpeedLerp.put(86.8261633902, 350.0);
  // }

  private static final DoubleUnaryOperator distanceToSpeed =
      meters -> 11.0169521624 * meters * meters + 277.447063272;
  private static final DoubleSupplier maxAllowableErrorRadPS =
      new TunableDouble(TUNING_NT_KEY + "/AutoAimAllowableError", 10, () -> true);
  private static double autoTargetSpeed = 0;

  public static Command hubAutoAim(
      ShooterSubsystem shooter,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d> targetSupplier) {
    return shooter.run(() -> {
      double targetDistance =
          robotPoseSupplier.get().getTranslation().getDistance(targetSupplier.get());
      autoTargetSpeed = distanceToSpeed.applyAsDouble(targetDistance);
      Logger.recordOutput(TUNING_NT_KEY + "/targetDistance", targetDistance);
      Logger.recordOutput(TUNING_NT_KEY + "/autoTargetSpeed", autoTargetSpeed);

      shooter.setRIOFlywheelGoal(new RadVel_State(autoTargetSpeed));
      shooter.setPDHFlywheelGoal(new RadVel_State(autoTargetSpeed));
    });
  }

  public static boolean isAutoAimReady(ShooterSubsystem shooter) {
    // if (!hubAutoAim(shooter, null, null).isScheduled()) {
    //   return false;
    // }
    return Math.abs(shooter.getRIOFlywheelState().vel(RadiansPerSecond) - autoTargetSpeed)
            < maxAllowableErrorRadPS.getAsDouble()
        && Math.abs(shooter.getPDHFlywheelState().vel(RadiansPerSecond) - autoTargetSpeed)
            < maxAllowableErrorRadPS.getAsDouble();
  }

  public static Command stop(ShooterSubsystem shooter) {
    return shooter.run(() -> {
      shooter.setHoodGoal(new RadVel_State(0));
      shooter.setRIOFlywheelGoal(new RadVel_State(0));
      shooter.setPDHFlywheelGoal(new RadVel_State(0));
    });
  }

  public static Command runForward(ShooterSubsystem shooter) {
    return shooter.run(() -> {
      shooter.setRIOFlywheelGoal(new RadVel_State(targetSpeed_radPs.getAsDouble()));
      shooter.setPDHFlywheelGoal(new RadVel_State(targetSpeed_radPs.getAsDouble()));
    });
  }

  public static Command raiseHood(ShooterSubsystem shooter) {
    return shooter.run(() -> {
      shooter.setHoodGoal(new RadVel_State(targetHoodSpeed_radPs.getAsDouble()));
    });
  }

  public static Command lowerHood(ShooterSubsystem shooter) {
    return shooter.run(() -> {
      shooter.setHoodGoal(new RadVel_State(-targetHoodSpeed_radPs.getAsDouble()));
    });
  }

  public static Command hoodSysId(ShooterSubsystem shooter, SysIdType type) {
    return Commands.run(() -> {}, shooter)
        .withTimeout(1)
        .andThen(shooter.getHoodSysIdCommand(type))
        .withName("HoodSysId_" + type.name());
  }

  public static Command rioFlywheelSysId(ShooterSubsystem shooter, SysIdType type) {
    return Commands.run(() -> {}, shooter)
        .withTimeout(1)
        .andThen(shooter.getRIOFlywheelSysIdCommand(type))
        .withName("FlywheelSysId_" + type.name());
  }

  public static Command pdhFlywheelSysId(ShooterSubsystem shooter, SysIdType type) {
    return Commands.run(() -> {}, shooter)
        .withTimeout(1)
        .andThen(shooter.getPDHFlywheelSysIdCommand(type))
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
