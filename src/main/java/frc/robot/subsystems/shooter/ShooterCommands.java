package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.logging.TunableBoolean;
import frc.robot.util.logging.TunableDouble;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterCommands {
  public static final boolean isTuning = false;
  private static final String TUNING_NT_KEY = "Tuning/" + LOG_NAME + "/Commands";
  private static final DoubleSupplier targetSpeed_radPs =
      new TunableDouble(TUNING_NT_KEY + "/Flywheel_radPs", 320, () -> true);
  private static final DoubleSupplier targetSpeed_radPs1 =
      new TunableDouble(TUNING_NT_KEY + "/Flywheel_radPs", 320, () -> true);
  private static final DoubleSupplier targetSpeed_radPs2 =
      new TunableDouble(TUNING_NT_KEY + "/Flywheel_radPs", 320, () -> true);
  // Change if flywheel accelerates too much during auto
  private static final DoubleSupplier targetHoodSpeed_radPs =
      new TunableDouble(TUNING_NT_KEY + "/Hood_radPs", .2, () -> true);
  // private static final InterpolatingDoubleTreeMap distanceToSpeedLerp =
  //     new InterpolatingDoubleTreeMap();

  // static {
  //   distanceToSpeedLerp.put(0.0, 0.0);
  //   distanceToSpeedLerp.put(86.8261633902, 350.0);
  // }
  public static double getFlywheelGoal() {
    return targetSpeed_radPs.getAsDouble();
  }

  public static void setAutoAimPoint(double distance_m) {
    hub_autoAimUsedValuesList.add(true);
    hub_autoAimDistancesList.add(distance_m);
    hub_autoAimFlywheelVelsList.add(targetSpeed_radPs.getAsDouble());
    hub_autoAimHoodPosesList.add(hood_goalPos.in(Radians));
    resetAutoAim();
    final int i = hub_autoAimUsedValuesList.size() - 1;
    new TunableBoolean(
        TUNING_NT_KEY + "/HubShooterTuningValues/" + i + "_"
            + distanceFormatter.format(hub_autoAimDistancesList.get(i)) + "m_"
            + flywheelFormatter.format(hub_autoAimFlywheelVelsList.get(i)) + "f_"
            + hoodFormatter.format(hub_autoAimHoodPosesList.get(i)) + "h",
        hub_autoAimUsedValuesList.get(i),
        () -> true,
        shouldUse -> {
          hub_autoAimUsedValuesList.set(i, shouldUse);
          resetAutoAim();
        });
    // passing_autoAimUsedValuesList.add(true);
    // passing_autoAimDistancesList.add(distance_m);
    // passing_autoAimFlywheelVelsList.add(targetSpeed_radPs.getAsDouble());
    // passing_autoAimHoodPosesList.add(hood_goalPos.in(Radians));
    // resetAutoAim();
    // final int i = passing_autoAimUsedValuesList.size() - 1;
    // new TunableBoolean(
    //     TUNING_NT_KEY + "/PassingShooterTuningValues/" + i + "_"
    //         + distanceFormatter.format(passing_autoAimDistancesList.get(i)) + "m_"
    //         + flywheelFormatter.format(passing_autoAimFlywheelVelsList.get(i)) + "f_"
    //         + hoodFormatter.format(passing_autoAimHoodPosesList.get(i)) + "h",
    //     passing_autoAimUsedValuesList.get(i),
    //     () -> true,
    //     shouldUse -> {
    //       passing_autoAimUsedValuesList.set(i, shouldUse);
    //       resetAutoAim();
    //     });
  }

  private static final InterpolatingDoubleTreeMap hub_metersToFywheelRadPerSec =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hub_metersToHoodRad =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passing_metersToFywheelRadPerSec =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passing_metersToHoodRad =
      new InterpolatingDoubleTreeMap();
  private static final List<Boolean> hub_autoAimUsedValuesList = new ArrayList<>();
  private static final List<Double> hub_autoAimDistancesList = new ArrayList<>();
  private static final List<Double> hub_autoAimFlywheelVelsList = new ArrayList<>();
  private static final List<Double> hub_autoAimHoodPosesList = new ArrayList<>();
  private static final List<Boolean> passing_autoAimUsedValuesList = new ArrayList<>();
  private static final List<Double> passing_autoAimDistancesList = new ArrayList<>();
  private static final List<Double> passing_autoAimFlywheelVelsList = new ArrayList<>();
  private static final List<Double> passing_autoAimHoodPosesList = new ArrayList<>();
  private static final NumberFormat longNumberFormatter = new DecimalFormat("###0.0########");
  private static final NumberFormat distanceFormatter = new DecimalFormat("###0.000");
  private static final NumberFormat flywheelFormatter = new DecimalFormat("###0.#");
  private static final NumberFormat hoodFormatter = new DecimalFormat("#.00#");

  private static void resetAutoAim() {
    hub_metersToFywheelRadPerSec.clear();
    hub_metersToHoodRad.clear();
    for (int i = 0; i < hub_autoAimUsedValuesList.size(); i++) {
      if (hub_autoAimUsedValuesList.get(i)) {
        hub_metersToFywheelRadPerSec.put(
            hub_autoAimDistancesList.get(i), hub_autoAimFlywheelVelsList.get(i));
        hub_metersToHoodRad.put(hub_autoAimDistancesList.get(i), hub_autoAimHoodPosesList.get(i));
      }
    }
    passing_metersToFywheelRadPerSec.clear();
    passing_metersToHoodRad.clear();
    for (int i = 0; i < passing_autoAimUsedValuesList.size(); i++) {
      if (passing_autoAimUsedValuesList.get(i)) {
        passing_metersToFywheelRadPerSec.put(
            passing_autoAimDistancesList.get(i), passing_autoAimFlywheelVelsList.get(i));
        passing_metersToHoodRad.put(
            passing_autoAimDistancesList.get(i), passing_autoAimHoodPosesList.get(i));
      }
    }
    if (isTuning) {
      String codeString = "";
      for (boolean bool : hub_autoAimUsedValuesList) {
        codeString += "   hub_autoAimUsedValuesList.add(" + (bool ? "true" : "false") + ");\r\n";
      }
      for (double dbl : hub_autoAimDistancesList) {
        codeString +=
            "   hub_autoAimDistancesList.add(" + longNumberFormatter.format(dbl) + ");\r\n";
      }
      for (double dbl : hub_autoAimFlywheelVelsList) {
        codeString +=
            "   hub_autoAimFlywheelVelsList.add(" + longNumberFormatter.format(dbl) + ");\r\n";
      }
      for (double dbl : hub_autoAimHoodPosesList) {
        codeString +=
            "   hub_autoAimHoodPosesList.add(" + longNumberFormatter.format(dbl) + ");\r\n";
      }
      for (boolean bool : passing_autoAimUsedValuesList) {
        codeString +=
            "   passing_autoAimUsedValuesList.add(" + (bool ? "true" : "false") + ");\r\n";
      }
      for (double dbl : passing_autoAimDistancesList) {
        codeString +=
            "   passing_autoAimDistancesList.add(" + longNumberFormatter.format(dbl) + ");\r\n";
      }
      for (double dbl : passing_autoAimFlywheelVelsList) {
        codeString +=
            "   passing_autoAimFlywheelVelsList.add(" + longNumberFormatter.format(dbl) + ");\r\n";
      }
      for (double dbl : passing_autoAimHoodPosesList) {
        codeString +=
            "   passing_autoAimHoodPosesList.add(" + longNumberFormatter.format(dbl) + ");\r\n";
      }
      SmartDashboard.putString("ShooterTuningCodeSnippet", codeString);
    }
  }

  static {
    // hub flywheel
    hub_autoAimUsedValuesList.add(true);
    hub_autoAimUsedValuesList.add(true);
    hub_autoAimUsedValuesList.add(true);
    hub_autoAimUsedValuesList.add(true);
    hub_autoAimUsedValuesList.add(true);
    hub_autoAimUsedValuesList.add(true);
    hub_autoAimUsedValuesList.add(false);
    hub_autoAimUsedValuesList.add(true);
    hub_autoAimUsedValuesList.add(true);
    hub_autoAimDistancesList.add(1.78377858);
    hub_autoAimDistancesList.add(1.307805649);
    hub_autoAimDistancesList.add(2.366279286);
    hub_autoAimDistancesList.add(2.843687808);
    hub_autoAimDistancesList.add(3.635341662);
    hub_autoAimDistancesList.add(4.33910244);
    hub_autoAimDistancesList.add(4.8904);
    hub_autoAimDistancesList.add(5.222222675);
    hub_autoAimDistancesList.add(4.86071509);
    hub_autoAimDistancesList.add(4.330782946);
    hub_autoAimFlywheelVelsList.add(300.0);
    hub_autoAimFlywheelVelsList.add(280.0);
    hub_autoAimFlywheelVelsList.add(310.0);
    hub_autoAimFlywheelVelsList.add(335.0);
    hub_autoAimFlywheelVelsList.add(365.0);
    hub_autoAimFlywheelVelsList.add(387.0);
    hub_autoAimFlywheelVelsList.add(403.0);
    hub_autoAimFlywheelVelsList.add(405.0);
    hub_autoAimFlywheelVelsList.add(390.0);
    hub_autoAimFlywheelVelsList.add(375.0);
    hub_autoAimHoodPosesList.add(0.000420749);
    hub_autoAimHoodPosesList.add(0.000420749);
    hub_autoAimHoodPosesList.add(0.128328668);
    hub_autoAimHoodPosesList.add(0.175032372);
    hub_autoAimHoodPosesList.add(0.21164);
    hub_autoAimHoodPosesList.add(0.2798);
    hub_autoAimHoodPosesList.add(0.29452);
    hub_autoAimHoodPosesList.add(0.323975844);
    hub_autoAimHoodPosesList.add(0.300414292);
    hub_autoAimHoodPosesList.add(0.261706002);
    passing_autoAimUsedValuesList.add(false);
    passing_autoAimUsedValuesList.add(false);
    passing_autoAimUsedValuesList.add(true);
    passing_autoAimUsedValuesList.add(true);
    passing_autoAimUsedValuesList.add(true);
    passing_autoAimUsedValuesList.add(true);
    passing_autoAimUsedValuesList.add(true);
    passing_autoAimDistancesList.add(3.417615538);
    passing_autoAimDistancesList.add(4.973876456);
    passing_autoAimDistancesList.add(2.140850699);
    passing_autoAimDistancesList.add(3.516237516);
    passing_autoAimDistancesList.add(6.002807498);
    passing_autoAimDistancesList.add(7.522956473);
    passing_autoAimDistancesList.add(9.0);
    passing_autoAimFlywheelVelsList.add(320.0);
    passing_autoAimFlywheelVelsList.add(380.0);
    passing_autoAimFlywheelVelsList.add(320.0);
    passing_autoAimFlywheelVelsList.add(380.0);
    passing_autoAimFlywheelVelsList.add(470.0);
    passing_autoAimFlywheelVelsList.add(550.0);
    passing_autoAimFlywheelVelsList.add(620.0);
    passing_autoAimHoodPosesList.add(0.5);
    passing_autoAimHoodPosesList.add(0.5);
    passing_autoAimHoodPosesList.add(0.5);
    passing_autoAimHoodPosesList.add(0.55);
    passing_autoAimHoodPosesList.add(0.61344649);
    passing_autoAimHoodPosesList.add(0.61344649);
    passing_autoAimHoodPosesList.add(0.61344649);

    if (isTuning) {
      for (int i = 0; i < hub_autoAimUsedValuesList.size(); i++) {
        final int index = i;
        new TunableBoolean(
            TUNING_NT_KEY + "/HubShooterTuningValues/" + i + "_"
                + distanceFormatter.format(hub_autoAimDistancesList.get(i)) + "m_"
                + flywheelFormatter.format(hub_autoAimFlywheelVelsList.get(i)) + "f_"
                + hoodFormatter.format(hub_autoAimHoodPosesList.get(i)) + "h",
            hub_autoAimUsedValuesList.get(i),
            () -> true,
            shouldUse -> {
              hub_autoAimUsedValuesList.set(index, shouldUse);
              resetAutoAim();
            });
      }
      for (int i = 0; i < passing_autoAimUsedValuesList.size(); i++) {
        final int index = i;
        new TunableBoolean(
            TUNING_NT_KEY + "/PassingShooterTuningValues/" + i + "_"
                + distanceFormatter.format(passing_autoAimDistancesList.get(i)) + "m_"
                + flywheelFormatter.format(passing_autoAimFlywheelVelsList.get(i)) + "f_"
                + hoodFormatter.format(passing_autoAimHoodPosesList.get(i)) + "h",
            passing_autoAimUsedValuesList.get(i),
            () -> true,
            shouldUse -> {
              passing_autoAimUsedValuesList.set(index, shouldUse);
              resetAutoAim();
            });
      }
    }
    resetAutoAim();
  }

  private static final DoubleSupplier maxAllowableErrorRadPS =
      new TunableDouble(TUNING_NT_KEY + "/AutoAimAllowableFlywheelError", 40, () -> true);
  private static final DoubleSupplier maxAllowableErrorRad =
      new TunableDouble(TUNING_NT_KEY + "/AutoAimAllowableHoodError", 0.03, () -> true);
  private static final Debouncer isReadyDebouncer = new Debouncer(.2, DebounceType.kFalling);
  private static final MutAngle hood_goalPos = Radians.mutable(0);
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
        hood_goalPos.mut_setMagnitude(passing_metersToHoodRad.get(targetDistance));
      } else {
        flywheel_autoAimVel.mut_setMagnitude(hub_metersToFywheelRadPerSec.get(targetDistance));
        hood_goalPos.mut_setMagnitude(hub_metersToHoodRad.get(targetDistance));
      }
      Logger.recordOutput("Commands/Shooter/targetDistance", targetDistance);
      Logger.recordOutput("Commands/Shooter/autoAimFlywheelVel", flywheel_autoAimVel);
      Logger.recordOutput("Commands/Shooter/autoAimHoodPos", hood_goalPos);

      if (trenchOverride.getAsBoolean()) {
        shooter.setHoodGoalPos(Radians.zero());
      } else {
        shooter.setHoodGoalPos(hood_goalPos);
      }
      shooter.setRIOFlywheelGoalVel(flywheel_autoAimVel);
      shooter.setPDHFlywheelGoalVel(flywheel_autoAimVel);
    });
  }

  public static boolean isAutoAimReady(ShooterSubsystem shooter, boolean isPassing) {
    return isReadyDebouncer.calculate(MathUtil.isNear(
            hood_goalPos.in(Radians),
            shooter.getHoodPos().in(Radians),
            maxAllowableErrorRad.getAsDouble())
        && ((MathUtil.isNear(
                    flywheel_autoAimVel.in(RadiansPerSecond),
                    shooter.getRIOFlywheelVel().in(RadiansPerSecond),
                    maxAllowableErrorRadPS.getAsDouble())
                && MathUtil.isNear(
                    flywheel_autoAimVel.in(RadiansPerSecond),
                    shooter.getPDHFlywheelVel().in(RadiansPerSecond),
                    maxAllowableErrorRadPS.getAsDouble()))
            || (isPassing
                && shooter.getRIOFlywheelAppliedOutput() > .9
                && shooter.getRIOFlywheelAppliedOutput() > .9)));
  }

  public static boolean isPassingReady(ShooterSubsystem shooter) {
    double batteryMult = 1 / (RobotController.getBatteryVoltage() / 13.0);
    if (batteryMult > 1.35) {
      batteryMult = 1.35;
    }
    ;
    double scalar = 1.25 * batteryMult;
    return isReadyDebouncer.calculate(MathUtil.isNear(
            hood_goalPos.in(Radians),
            shooter.getHoodPos().in(Radians),
            maxAllowableErrorRad.getAsDouble())
        && MathUtil.isNear(
            flywheel_autoAimVel.in(RadiansPerSecond),
            shooter.getRIOFlywheelVel().in(RadiansPerSecond),
            maxAllowableErrorRadPS.getAsDouble() * scalar)
        && MathUtil.isNear(
            flywheel_autoAimVel.in(RadiansPerSecond),
            shooter.getPDHFlywheelVel().in(RadiansPerSecond),
            maxAllowableErrorRadPS.getAsDouble() * scalar));
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
          hood_goalPos.mut_setMagnitude(shooter.getHoodPos().in(Radians));
          return shooter.run(() -> {
            shooter.setHoodGoalPos(hood_goalPos);
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

  public static Command runForward1(ShooterSubsystem shooter) {
    return shooter.run(() -> {
      shooter.setRIOFlywheelGoalVel(RadiansPerSecond.of(targetSpeed_radPs1.getAsDouble()));
      shooter.setPDHFlywheelGoalVel(RadiansPerSecond.of(targetSpeed_radPs1.getAsDouble()));
    });
  }

  public static Command runForward2(ShooterSubsystem shooter) {
    return shooter.run(() -> {
      shooter.setRIOFlywheelGoalVel(RadiansPerSecond.of(targetSpeed_radPs2.getAsDouble()));
      shooter.setPDHFlywheelGoalVel(RadiansPerSecond.of(targetSpeed_radPs2.getAsDouble()));
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
