// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Robot;
import frc.robot.util.UnitUtil;
import frc.robot.util.logging.LogUtil;
import frc.robot.util.nova.NovaConfig;
import frc.robot.util.nova.NovaConfig.BrakeMode;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class DriveConstants {
  public static final String name = "Drive";
  public static final LoggedNetworkBoolean tunable =
      new LoggedNetworkBoolean("Tunable/" + name + "/.Tunable", false);

  public static final class Swerve {
    public static final String name = DriveConstants.name + "/Swerve";

    // physical properties
    public static final double bumperLength_m = UnitUtil.inTom(30.625);
    public static final double bumperWidth_m = UnitUtil.inTom(30.625);
    public static final double trackLength_m = UnitUtil.inTom(21.625 - (2 * 1.6875));
    public static final double trackWidth_m = UnitUtil.inTom(21.625 - (2 * 1.6875));
    public static final double trackRadius_m = Math.hypot(trackLength_m / 2.0, trackWidth_m / 2.0);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
      new Translation2d(trackLength_m / 2.0, trackWidth_m / 2.0),
      new Translation2d(trackLength_m / 2.0, -trackWidth_m / 2.0),
      new Translation2d(-trackLength_m / 2.0, trackWidth_m / 2.0),
      new Translation2d(-trackLength_m / 2.0, -trackWidth_m / 2.0)
    };

    // software limits
    public static final double odometryFrequency_Hz = 100.0;

    public static final RobotConfig ppConfig = new RobotConfig(
        Robot.mass_kg,
        Robot.moi_kgm2,
        new ModuleConfig(
            Wheel.radius_m,
            Wheel.max_mPs,
            Wheel.cof,
            Wheel.gearbox.withReduction(Wheel.reduction),
            Wheel.config.limits.getMaxStatorCurrent(),
            1),
        Swerve.moduleTranslations);

    public static final DriveTrainSimulationConfig mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withBumperSize(Meters.of(bumperLength_m), Meters.of(bumperWidth_m))
            .withCustomModuleTranslations(Swerve.moduleTranslations)
            .withRobotMass(Kilogram.of(Robot.mass_kg))
            .withGyro(COTS.ofNav2X())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                Wheel.gearbox,
                Azimuth.gearbox,
                Wheel.reduction,
                Azimuth.reduction,
                Volts.of(Wheel.realFF.getKs()),
                Volts.of(Azimuth.realFF.getKs()),
                Meters.of(Wheel.radius_m),
                KilogramSquareMeters.of(Azimuth.moi_kgm2),
                Wheel.cof));
  }

  public static class Wheel {
    public static final String name = DriveConstants.name + "/Wheel";

    // physical properties
    public static final double radius_m = Units.inchesToMeters((3.875 - .12) / 2);
    public static final double cof = 1.2;
    /** FL, FR, BL, BR */
    public static final int[] canIds = new int[] {3, 5, 1, 7};

    public static final double reduction = 6.25666667;
    public static final DCMotor gearbox = DCMotor.getNeoVortex(1);

    // software limits
    public static final double max_mPs = Units.feetToMeters(15);
    public static final NovaConfig config = new NovaConfig();

    // Drive PID configuration
    public static final SimpleMotorFeedforward
        realFF = new SimpleMotorFeedforward(0.013, 0.01, 0.0, Robot.codePeriod_s),
        simFF = new SimpleMotorFeedforward(0.036968, 0.15869, 0.034, Robot.codePeriod_s);
    public static final PIDController
        realPID = new PIDController(0.0, 0.0, 0.0, Robot.codePeriod_s),
        simPID = new PIDController(0.23931, 0.0, 0.0, Robot.codePeriod_s);

    static {
      config.setBrakeMode(BrakeMode.BRAKE).setVoltageCompensation(Robot.nominal_V);
      config.limits.setMaxStatorCurrent(60).setMaxSupplyCurrent(60);
      config
          .canFreq
          .setSensorPeriod(1 / Swerve.odometryFrequency_Hz)
          .setControlPeriod(0.02)
          .setCurrentPeriod(0.02)
          .setFaultPeriod(0.02);

      LogUtil.createTunablePID("Tunable/" + name + "/real", realPID, tunable::get);
      LogUtil.createTunableFF("Tunable/" + name + "/real", realFF, tunable::get);
      LogUtil.createTunablePID("Tunable/" + name + "/sim", simPID, tunable::get);
      LogUtil.createTunableFF("Tunable/" + name + "/sim", simFF, tunable::get);
    }
  }

  public static class Azimuth {
    public static final String name = DriveConstants.name + "/Azimuth";

    // physical properties
    public static final double moi_kgm2 = 0.02;
    /** FL, FR, BL, BR */
    public static final int[] canIds = new int[] {4, 6, 2, 8};

    public static final double reduction = 25;
    public static final DCMotor gearbox = DCMotor.getNEO(1);

    // software limits
    public static final NovaConfig config = new NovaConfig();

    public static final SimpleMotorFeedforward
        realFF = new SimpleMotorFeedforward(0.01, 0.051, 0.0, Robot.codePeriod_s),
        simFF = new SimpleMotorFeedforward(0.004, 0.4960674, 0.006, Robot.codePeriod_s);
    public static final PIDController
        realPID = new PIDController(0.5, 0.0, 0.0, Robot.codePeriod_s),
        simPID = new PIDController(4, 0.0, 0.05, Robot.codePeriod_s);

    static {
      config
          .setInversion(false)
          .setBrakeMode(BrakeMode.BRAKE)
          .setVoltageCompensation(Robot.nominal_V);
      config.limits.setMaxStatorCurrent(50).setMaxSupplyCurrent(20);
      config
          .canFreq
          .setSensorPeriod(1 / Swerve.odometryFrequency_Hz)
          .setControlPeriod(0.02)
          .setCurrentPeriod(0.02)
          .setFaultPeriod(0.02);

      realPID.enableContinuousInput(0, 2 * Math.PI);
      simPID.enableContinuousInput(0, 2 * Math.PI);
      LogUtil.createTunablePID("Tunable/" + name + "/real", realPID, tunable::get);
      LogUtil.createTunableFF("Tunable/" + name + "/real", realFF, tunable::get);
      LogUtil.createTunablePID("Tunable/" + name + "/sim", simPID, tunable::get);
      LogUtil.createTunableFF("Tunable/" + name + "/sim", simFF, tunable::get);
    }
  }

  public static class AbsEncoder {
    public static final String name = DriveConstants.name + "/AbsEncoder";

    /** FL, FR, BL, BR */
    public static final int[] analogPorts = new int[] {0, 1, 2, 3};

    /** FL, FR, BL, BR. Rotation of each absolute encoder when the wheels face forward */
    public static final double[] zeroRotations_rad =
        new double[] {4.26675, 2.99095, 2.40695, 3.355259};
  }
}
