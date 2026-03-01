package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerCommands;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.logging.TunableDouble;
import frc.robot.util.states.premade.RadVel_State;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShooterCommands {
  private static final DoubleSupplier targetSpeed_radPs =
      new TunableDouble(FLYWHEEL_CONSTANTS.tuningLogName + "/_radPs", 471, () -> true);
  private static final DoubleSupplier targetHoodSpeed_radPs =
      new TunableDouble(HOOD_CONSTANTS.tuningLogName + "/_radPs", .2, () -> true);
  private static final InterpolatingDoubleTreeMap distanceToSpeedLerp =
      new InterpolatingDoubleTreeMap();

  static {
    distanceToSpeedLerp.put(0.0, 0.0);
    distanceToSpeedLerp.put(3.0, 350.0);
  }

  private static final Translation2d blueHub = new Translation2d(0, 0);
  private static final Translation2d redHub = new Translation2d(0, 0);
  //
  public static Command autoAim(Supplier<Pose2d> robotPoseSupplier, ShooterSubsystem shooter) {
    return shooter.run(() -> {
      double distance;
      if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
        distance = blueHub.minus(robotPoseSupplier.get().getTranslation()).getNorm();
      } else {
        distance = redHub.minus(robotPoseSupplier.get().getTranslation()).getNorm();
      }
      shooter.setFlywheelGoal(new RadVel_State(distanceToSpeedLerp.get(distance)));
    });
  }
  // placeholder/prototype shooter -> indexer + autoAim commands:
  /* 
  public static Command AutoFireFuel(Supplier<Pose2d> robotPoseSupplier, ShooterSubsystem shooter, IndexerSubsystem indexer) {
    double distance;
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
        distance = blueHub.minus(robotPoseSupplier.get().getTranslation()).getNorm();
      } else {
        distance = redHub.minus(robotPoseSupplier.get().getTranslation()).getNorm();
      }
    //
    return Commands.parallel(
    shooter.run(() -> {
      shooter.setFlywheelGoal(new RadVel_State(distanceToSpeedLerp.get(distance)));
    }),
    indexer.run(() -> {
      indexer.setHopperGoal(new RadVel_State(IndexerCommands.forward_radPs.getAsDouble()));
      indexer.setKickerGoal(new RadVel_State(IndexerCommands.forward_radPs.getAsDouble()));
    }).unless(//measured flywheel velocity < distanceToSpeedLerp.get(distance)));
  }*/

  //
  public static Command stop(ShooterSubsystem shooter) {
    return shooter.run(() -> {
      shooter.setFlywheelGoal(new RadVel_State(0));
      shooter.setHoodGoal(new RadVel_State(0));
    });
  }

  public static Command runForward(ShooterSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setFlywheelGoal(new RadVel_State(targetSpeed_radPs.getAsDouble()));
        },
        instance);
  }

  public static Command raiseHood(ShooterSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setHoodGoal(new RadVel_State(targetHoodSpeed_radPs.getAsDouble()));
        },
        instance);
  }

  public static Command lowerHood(ShooterSubsystem instance) {
    return Commands.run(
        () -> {
          instance.setHoodGoal(new RadVel_State(-targetHoodSpeed_radPs.getAsDouble()));
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
