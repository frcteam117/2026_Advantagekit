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

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.thethriftybot.devices.ThriftyNova.CurrentType;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;
import frc.robot.util.UnitUtil;
import frc.robot.util.logging.LogUtil;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class DrivetrainConstants {
  public static final String NAME = "1_Drivetrain";
  public static final LoggedNetworkBoolean tunable =
      new LoggedNetworkBoolean(RobotConstants.TUNING_PREFIX + NAME + "/.Tunable", false);

  public static final class Chassis {
    public static final String name = DrivetrainConstants.NAME + "/Chassis";

    // physical properties
    public static final double bumperLength_m = UnitUtil.inTom(30.625);
    public static final double bumperWidth_m = UnitUtil.inTom(30.625);
    public static final double trackLength_m = UnitUtil.inTom(21.625 - (2 * 1.6875));
    public static final double trackWidth_m = UnitUtil.inTom(21.625 - (2 * 1.6875) + 2);
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
        RobotConstants.MASS_kg,
        RobotConstants.MOI_kgm2,
        new ModuleConfig(
            Drive.radius_m,
            Drive.max_mPs,
            Drive.cof,
            Drive.gearbox.withReduction(Drive.reduction),
            Drive.config.maxCurrent,
            1),
        Chassis.moduleTranslations);

    public static final DriveTrainSimulationConfig mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withBumperSize(Meters.of(bumperLength_m), Meters.of(bumperWidth_m))
            .withCustomModuleTranslations(Chassis.moduleTranslations)
            .withRobotMass(Kilogram.of(RobotConstants.MASS_kg))
            .withGyro(COTS.ofNav2X())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                Drive.gearbox,
                Azimuth.gearbox,
                Drive.reduction,
                Azimuth.reduction,
                Volts.of(Drive.realFF.getKs()),
                Volts.of(Azimuth.realFF.getKs()),
                Meters.of(Drive.radius_m),
                KilogramSquareMeters.of(Azimuth.moi_kgm2),
                Drive.cof));
  }

  public static class Drive {
    public static final String name = DrivetrainConstants.NAME + "/Drive";

    // physical properties
    public static final double radius_m = Units.inchesToMeters((3.875 - .12) / 2);
    public static final double cof = 1.2;
    /** FL, FR, BL, BR */
    public static final int[] canIds = new int[] {3, 5, 1, 7};

    public static final double reduction = 6.25666667;
    public static final DCMotor gearbox = DCMotor.getNeoVortex(1);

    // software limits
    public static final double max_mPs = Units.feetToMeters(15);
    public static final ThriftyNovaConfig config = new ThriftyNovaConfig();

    // Drive PID configuration
    public static final SimpleMotorFeedforward
        realFF = new SimpleMotorFeedforward(0.013, 0.01, 0.0, RobotConstants.CODE_PERIOD_s),
        simFF = new SimpleMotorFeedforward(0.036968, 0.15869, 0.034, RobotConstants.CODE_PERIOD_s);
    public static final PIDController
        realPID = new PIDController(0.0, 0.0, 0.0, RobotConstants.CODE_PERIOD_s),
        simPID = new PIDController(0.23931, 0.0, 0.0, RobotConstants.CODE_PERIOD_s);

    static {
      config.brakeMode = false;
      config.voltageCompensation = RobotConstants.NOMINAL_V;
      config.currentType = CurrentType.STATOR;
      config.maxCurrent = 30.0;
      config.canFreq.sensor = 1 / Chassis.odometryFrequency_Hz;
      config.canFreq.control = 0.02;
      config.canFreq.current = 0.02;
      config.canFreq.fault = 0.02;

      LogUtil.createTunablePID(
          RobotConstants.TUNING_PREFIX + name + "/real", realPID, tunable::get);
      LogUtil.createTunableFF(RobotConstants.TUNING_PREFIX + name + "/real", realFF, tunable::get);
      LogUtil.createTunablePID(RobotConstants.TUNING_PREFIX + name + "/sim", simPID, tunable::get);
      LogUtil.createTunableFF(RobotConstants.TUNING_PREFIX + name + "/sim", simFF, tunable::get);
    }
  }

  public static class Azimuth {
    public static final String name = DrivetrainConstants.NAME + "/Azimuth";

    // physical properties
    public static final double moi_kgm2 = 0.02;
    /** FL, FR, BL, BR */
    public static final int[] canIds = new int[] {4, 6, 2, 8};

    public static final double reduction = 25;
    public static final DCMotor gearbox = DCMotor.getNEO(1);

    // software limits
    public static final ThriftyNovaConfig config = new ThriftyNovaConfig();

    public static final SimpleMotorFeedforward
        realFF = new SimpleMotorFeedforward(0.01, 0.051, 0.0, RobotConstants.CODE_PERIOD_s),
        simFF = new SimpleMotorFeedforward(0.004, 0.4960674, 0.006, RobotConstants.CODE_PERIOD_s);
    public static final PIDController
        realPID = new PIDController(0.5, 0.0, 0.0, RobotConstants.CODE_PERIOD_s),
        simPID = new PIDController(4, 0.0, 0.05, RobotConstants.CODE_PERIOD_s);

    static {
      config.brakeMode = false;
      config.voltageCompensation = RobotConstants.NOMINAL_V;
      config.currentType = CurrentType.STATOR;
      config.maxCurrent = 30.0;
      config.canFreq.sensor = Chassis.odometryFrequency_Hz;
      config.canFreq.control = 50.0;
      config.canFreq.current = 50.0;
      config.canFreq.fault = 50.0;

      realPID.enableContinuousInput(0, 2 * Math.PI);
      simPID.enableContinuousInput(0, 2 * Math.PI);
      LogUtil.createTunablePID(
          RobotConstants.TUNING_PREFIX + name + "/real", realPID, tunable::get);
      LogUtil.createTunableFF(RobotConstants.TUNING_PREFIX + name + "/real", realFF, tunable::get);
      LogUtil.createTunablePID(RobotConstants.TUNING_PREFIX + name + "/sim", simPID, tunable::get);
      LogUtil.createTunableFF(RobotConstants.TUNING_PREFIX + name + "/sim", simFF, tunable::get);
    }
  }

  public static class AbsEncoder {
    public static final String name = DrivetrainConstants.NAME + "/AbsEncoder";

    /** FL, FR, BL, BR */
    public static final int[] analogPorts = new int[] {0, 1, 2, 3};

    /** FL, FR, BL, BR. Rotation of each absolute encoder when the wheels face forward */
    public static final double[] zeroRotations_rad =
        new double[] {4.26675, 2.99095, 2.40695, 3.355259};
  }
}
