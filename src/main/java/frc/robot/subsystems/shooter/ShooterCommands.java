package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.UnitUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShooterCommands {
  public static Command flywheelSysId(Shooter shooter, SysIdType type) {
    return Commands.run(shooter::stop, shooter)
        .withTimeout(1)
        .andThen(shooter.getFlywheelSysId(type))
        .withName("FlywheelSysId_" + type.name());
  }

  public static Command hoodSysId(Shooter shooter, SysIdType type) {
    return Commands.run(shooter::stop, shooter)
        .withTimeout(1)
        .andThen(shooter.getHoodSysId(type))
        .withName("HoodSysId_" + type.name());
  }

  public static Command turretSysId(Shooter shooter, SysIdType type) {
    return Commands.run(shooter::stop, shooter)
        .withTimeout(1)
        .andThen(shooter.getTurretSysId(type))
        .withName("TurretSysId_" + type.name());
  }

  public static Command runFlywheelVoltage(Shooter shooter, DoubleSupplier supplier_V) {
    return Commands.run(() -> shooter.setFlywheelVoltage(supplier_V.getAsDouble()), shooter);
  }

  public static Command runFlywheelGoalVelocity(Shooter shooter, DoubleSupplier supplier_radPs) {
    return Commands.run(
        () -> shooter.setFlywheelGoalVelocity(supplier_radPs.getAsDouble()), shooter);
  }

  public static Command runHoodVoltage(Shooter shooter, DoubleSupplier supplier_V) {
    return Commands.run(() -> shooter.setHoodVoltage(supplier_V.getAsDouble()), shooter);
  }

  public static Command runHoodGoalState(
      Shooter shooter, DoubleSupplier supplier_rad, DoubleSupplier supplier_radPs) {
    return Commands.run(
        () -> shooter.setHoodGoalState(supplier_rad.getAsDouble(), supplier_radPs.getAsDouble()),
        shooter);
  }

  public static Command runHoodGoalPosition(Shooter shooter, DoubleSupplier supplier_rad) {
    return runHoodGoalState(shooter, supplier_rad, () -> 0);
  }

  public static Command runTurretVoltage(Shooter shooter, DoubleSupplier supplier_V) {
    return Commands.run(() -> shooter.setTurretVoltage(supplier_V.getAsDouble()), shooter);
  }

  public static Command runTurretGoalState(
      Shooter shooter, DoubleSupplier supplier_rad, DoubleSupplier supplier_radPs) {
    return Commands.run(
        () -> shooter.setTurretGoalState(supplier_rad.getAsDouble(), supplier_radPs.getAsDouble()),
        shooter);
  }

  public static Command runTurretGoalPosition(Shooter shooter, DoubleSupplier supplier_rad) {
    return runTurretGoalState(shooter, supplier_rad, () -> 0);
  }

  public static Command trackPosition(
      Shooter shooter, Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> targetSupplier) {
    return Commands.run(
        () -> {
          Translation2d robotToTarget =
              targetSupplier.get().minus(robotPoseSupplier.get().getTranslation());
          shooter.setTurretGoalPosition(robotToTarget.getAngle().getRadians()
              - robotPoseSupplier.get().getRotation().getRadians());
          double pitch_rad = Math.atan(UnitUtil.ftTom(16) / robotToTarget.getNorm());
          shooter.setHoodGoalPosition(pitch_rad);
          shooter.setFlywheelGoalVelocity(
              UnitUtil.ftTom(16) / Math.sin(pitch_rad) / UnitUtil.inTom(2));
        },
        shooter);
  }
}
