package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.logging.TunableDouble;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
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

  private static final InterpolatingDoubleTreeMap hub_metersToFywheelRadPerSec =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hub_metersToHoodRad =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passing_metersToFywheelRadPerSec =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passing_metersToHoodRad =
      new InterpolatingDoubleTreeMap();

  static {
    // hub flywheel
    hub_metersToFywheelRadPerSec.put(1.4890792, 270.0);
    hub_metersToFywheelRadPerSec.put(1.6669853, 285.0);
    hub_metersToFywheelRadPerSec.put(1.8409906, 300.0);
    hub_metersToFywheelRadPerSec.put(2.0387364, 315.0);
    hub_metersToFywheelRadPerSec.put(2.1137557, 320.0);
    hub_metersToFywheelRadPerSec.put(2.2659598, 330.0);
    hub_metersToFywheelRadPerSec.put(2.5085782, 328.0);
    hub_metersToFywheelRadPerSec.put(2.7848045, 340.0);
    hub_metersToFywheelRadPerSec.put(3.2773446, 352.0);
    hub_metersToFywheelRadPerSec.put(4.0661689, 376.0);
    hub_metersToFywheelRadPerSec.put(4.7832148, 403.0);
    hub_metersToFywheelRadPerSec.put(5.403332, 430.0);
    // hub hood
    hub_metersToHoodRad.put(1.4890792, 0.0);
    hub_metersToHoodRad.put(1.6669853, 0.0);
    hub_metersToHoodRad.put(1.8409906, 0.0);
    hub_metersToHoodRad.put(2.0387364, 0.0);
    hub_metersToHoodRad.put(2.1137557, 0.056);
    hub_metersToHoodRad.put(2.2659598, 0.062);
    hub_metersToHoodRad.put(2.5085782, 0.099);
    hub_metersToHoodRad.put(2.7848045, 0.13);
    hub_metersToHoodRad.put(3.2773446, 0.172);
    hub_metersToHoodRad.put(4.0661689, 0.221);
    hub_metersToHoodRad.put(4.7832148, 0.257);
    hub_metersToHoodRad.put(5.403332, 0.295);
    // passing flywheel
    passing_metersToFywheelRadPerSec.put(1.4890792, 270.0);
    passing_metersToFywheelRadPerSec.put(1.6669853, 285.0);
    passing_metersToFywheelRadPerSec.put(1.8409906, 300.0);
    passing_metersToFywheelRadPerSec.put(2.0387364, 315.0);
    passing_metersToFywheelRadPerSec.put(2.1137557, 320.0);
    passing_metersToFywheelRadPerSec.put(2.2659598, 330.0);
    passing_metersToFywheelRadPerSec.put(2.5085782, 328.0);
    passing_metersToFywheelRadPerSec.put(2.7848045, 340.0);
    passing_metersToFywheelRadPerSec.put(3.2773446, 352.0);
    passing_metersToFywheelRadPerSec.put(4.0661689, 376.0);
    passing_metersToFywheelRadPerSec.put(4.7832148, 403.0);
    passing_metersToFywheelRadPerSec.put(5.403332, 430.0);
    // passing hood
    passing_metersToHoodRad.put(1.4890792, 0.0);
    passing_metersToHoodRad.put(1.6669853, 0.0);
    passing_metersToHoodRad.put(1.8409906, 0.0);
    passing_metersToHoodRad.put(2.0387364, 0.0);
    passing_metersToHoodRad.put(2.1137557, 0.056);
    passing_metersToHoodRad.put(2.2659598, 0.062);
    passing_metersToHoodRad.put(2.5085782, 0.099);
    passing_metersToHoodRad.put(2.7848045, 0.13);
    passing_metersToHoodRad.put(3.2773446, 0.172);
    passing_metersToHoodRad.put(4.0661689, 0.221);
    passing_metersToHoodRad.put(4.7832148, 0.257);
    passing_metersToHoodRad.put(5.403332, 0.295);
  }

  private static final DoubleSupplier maxAllowableErrorRadPS =
      new TunableDouble(TUNING_NT_KEY + "/AutoAimAllowableFlywheelError", 30, () -> true);
  private static final DoubleSupplier maxAllowableErrorRad =
      new TunableDouble(TUNING_NT_KEY + "/AutoAimAllowableHoodError", 0.03, () -> true);
  private static final MutAngle hood_autoAimPos = Radians.mutable(0);
  // private static final MutAngularVelocity hood_autoAimVel = RadiansPerSecond.mutable(0);
  private static final MutAngularVelocity flywheel_autoAimVel = RadiansPerSecond.mutable(0);
  // private static final MutAngularAcceleration flywheel_autoAimAcc =
  // RadiansPerSecondPerSecond.mutable(0);

  public static Command autoAim(
      ShooterSubsystem shooter,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d> targetSupplier,
      BooleanSupplier passing,
      BooleanSupplier trenchOverride) {
    return shooter.run(() -> {
      final double targetDistance =
          robotPoseSupplier.get().getTranslation().getDistance(targetSupplier.get());
      if (passing.getAsBoolean()) {
        flywheel_autoAimVel.mut_setMagnitude(passing_metersToFywheelRadPerSec.get(targetDistance));
        hood_autoAimPos.mut_setMagnitude(passing_metersToHoodRad.get(targetDistance));
      } else {
        flywheel_autoAimVel.mut_setMagnitude(hub_metersToFywheelRadPerSec.get(targetDistance));
        hood_autoAimPos.mut_setMagnitude(hub_metersToHoodRad.get(targetDistance));
      }
      Logger.recordOutput("Commands/Shooter/targetDistance", targetDistance);
      Logger.recordOutput("Commands/Shooter/autoAimFlywheelVel", flywheel_autoAimVel);
      Logger.recordOutput("Commands/Shooter/autoAimHoodPos", hood_autoAimPos);

      if (trenchOverride.getAsBoolean()) {
        shooter.setHoodGoalPos(Radians.zero());
      } else {
        shooter.setHoodGoalPos(hood_autoAimPos);
      }
      shooter.setRIOFlywheelGoalVel(flywheel_autoAimVel);
      shooter.setPDHFlywheelGoalVel(flywheel_autoAimVel);
    });
  }

  public static boolean isAutoAimReady(ShooterSubsystem shooter) {
    return MathUtil.isNear(
            hood_autoAimPos.in(Radians),
            shooter.getHoodPos().in(Radians),
            maxAllowableErrorRad.getAsDouble())
        && MathUtil.isNear(
            flywheel_autoAimVel.in(RadiansPerSecond),
            shooter.getRIOFlywheelVel().in(RadiansPerSecond),
            maxAllowableErrorRadPS.getAsDouble())
        && MathUtil.isNear(
            flywheel_autoAimVel.in(RadiansPerSecond),
            shooter.getPDHFlywheelVel().in(RadiansPerSecond),
            maxAllowableErrorRadPS.getAsDouble());
  }

  public static Command stopAndZeroHood(ShooterSubsystem shooter) {
    return shooter.run(() -> {
      shooter.setHoodGoalPos(Radians.zero());
      shooter.setRIOFlywheelGoalVel(RadiansPerSecond.zero());
      shooter.setPDHFlywheelGoalVel(RadiansPerSecond.zero());
    });
  }

  public static Command stopAndHoldHood(ShooterSubsystem shooter) {
    return Commands.defer(
        () -> {
          Angle hoodTarget = shooter.getHoodPos();
          return shooter.run(() -> {
            shooter.setHoodGoalPos(hoodTarget);
            shooter.setRIOFlywheelGoalVel(RadiansPerSecond.zero());
            shooter.setPDHFlywheelGoalVel(RadiansPerSecond.zero());
          });
        },
        Set.of(shooter));
  }

  public static Command runForward(ShooterSubsystem shooter) {
    return shooter.run(() -> {
      shooter.setRIOFlywheelGoalVel(RadiansPerSecond.of(targetSpeed_radPs.getAsDouble()));
      shooter.setPDHFlywheelGoalVel(RadiansPerSecond.of(targetSpeed_radPs.getAsDouble()));
    });
  }

  public static Command raiseHood(ShooterSubsystem shooter) {
    return shooter.run(() -> {
      shooter.setHoodGoalVel(RadiansPerSecond.of(0.1));
    });
  }

  public static Command lowerHood(ShooterSubsystem shooter) {
    return shooter.run(() -> {
      shooter.setHoodGoalVel(RadiansPerSecond.of(-0.1));
    });
  }

  // public static Command hoodSysId(ShooterSubsystem shooter, SysIdType type) {
  //   return Commands.run(() -> {}, shooter)
  //       .withTimeout(1)
  //       .andThen(shooter.getHoodSysIdCommand(type))
  //       .withName("HoodSysId_" + type.name());
  // }

  // public static Command rioFlywheelSysId(ShooterSubsystem shooter, SysIdType type) {
  //   return Commands.run(() -> {}, shooter)
  //       .withTimeout(1)
  //       .andThen(shooter.getRIOFlywheelSysIdCommand(type))
  //       .withName("FlywheelSysId_" + type.name());
  // }

  // public static Command pdhFlywheelSysId(ShooterSubsystem shooter, SysIdType type) {
  //   return Commands.run(() -> {}, shooter)
  //       .withTimeout(1)
  //       .andThen(shooter.getPDHFlywheelSysIdCommand(type))
  //       .withName("FlywheelSysId_" + type.name());
  // }

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
