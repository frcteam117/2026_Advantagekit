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

import com.pathplanner.lib.config.RobotConfig;
import com.thethriftybot.devices.ThriftyNova.CurrentType;
import com.thethriftybot.devices.ThriftyNova.EncoderType;
import com.thethriftybot.devices.ThriftyNova.ExternalEncoder;
import com.thethriftybot.devices.ThriftyNova.PIDSlot;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
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
    public static final double trackLength_m = UnitUtil.inTom(21.625 - 1.6875); // 19.9375
    public static final double trackWidth_m = UnitUtil.inTom(25.625 - 1.6875); // 23.9375
    public static final double bumperLength_m = trackLength_m + UnitUtil.inTom(6);
    public static final double bumperWidth_m = trackWidth_m + UnitUtil.inTom(6);
    public static final double trackRadius_m = Math.hypot(trackLength_m / 2.0, trackWidth_m / 2.0);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
      new Translation2d(trackLength_m / 2.0, trackWidth_m / 2.0),
      new Translation2d(trackLength_m / 2.0, -trackWidth_m / 2.0),
      new Translation2d(-trackLength_m / 2.0, trackWidth_m / 2.0),
      new Translation2d(-trackLength_m / 2.0, -trackWidth_m / 2.0)
    };
    public static final Translation2d cmPosition = new Translation2d(0.0, 0.0);

    // software limits
    public static final double odometryFrequency_Hz = 100.0;

    public static final RobotConfig ppConfig;

    static {
      RobotConfig tempConfig = null;
      try {
        tempConfig = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      }
      ppConfig = tempConfig;
    }
    //  = new RobotConfig(
    //     RobotConstants.MASS_kg,
    //     RobotConstants.MOI_kgm2,
    //     new ModuleConfig(
    //         Drive.radius_m, Drive.max_mPs, Drive.cof, Drive.gearbox, Drive.config.maxCurrent, 1),
    //     Chassis.moduleTranslations);

    public static final DriveTrainSimulationConfig mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withBumperSize(Meters.of(bumperLength_m), Meters.of(bumperWidth_m))
            .withCustomModuleTranslations(Chassis.moduleTranslations)
            .withRobotMass(Kilogram.of(RobotConstants.MASS_kg))
            .withGyro(COTS.ofNav2X())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                DCMotor.getNeoVortex(1),
                DCMotor.getNEO(1),
                Drive.reduction,
                Azimuth.reduction,
                Volts.of(Drive.realFF.getKs()),
                Volts.of(Azimuth.realFF.getKs()),
                Meters.of(Drive.radius_m),
                KilogramSquareMeters.of(Azimuth.moi_kgm2),
                Drive.cof));
    // public static final DriveTrainSimulationConfig mapleSimConfig =
    //     DriveTrainSimulationConfig.Default()
    //         .withBumperSize(Meters.of(bumperLength_m), Meters.of(bumperWidth_m))
    //         .withCustomModuleTranslations(Chassis.moduleTranslations)
    //         .withRobotMass(Kilogram.of(40))
    //         .withGyro(COTS.ofNav2X())
    //         .withSwerveModule(new SwerveModuleSimulationConfig(
    //             DCMotor.getNeoVortex(1),
    //             DCMotor.getNEO(1),
    //             Drive.reduction,
    //             Azimuth.reduction,
    //             Volts.of(0.02),
    //             Volts.of(0.02),
    //             Meters.of(Drive.radius_m),
    //             KilogramSquareMeters.of(0.08),
    //             1.2));
  }

  public static class Drive {
    public static final String name = DrivetrainConstants.NAME + "/Drive";

    // physical properties
    public static final double radius_m =
        0.050; // 0.0476885; // Units.inchesToMeters((3.875 - .12) / 2);
    public static final double cof = 1.2;
    /** FL, FR, BL, BR */
    public static final int[] canIds = new int[] {7, 1, 5, 3};

    public static final double reduction = 6.23;
    public static final DCMotor gearbox = DCMotor.getNeoVortex(1).withReduction(reduction);

    // software limits
    public static final double max_mPs = Units.feetToMeters(15);
    public static final ThriftyNovaConfig config = new ThriftyNovaConfig();

    // Drive PID configuration
    public static final SimpleMotorFeedforward
        realFF = new SimpleMotorFeedforward(0.02, 0.011, 0.0008, RobotConstants.CODE_PERIOD_s),
        simFF = new SimpleMotorFeedforward(0.036968, 0.15869, 0.034, RobotConstants.CODE_PERIOD_s);
    public static final PIDController
        realPID = new PIDController(0.001, 0.0, 0.0, RobotConstants.CODE_PERIOD_s),
        simPID = new PIDController(0.23931, 0.0, 0.0, RobotConstants.CODE_PERIOD_s);

    static {
      config.brakeMode = true;
      config.voltageCompensation = RobotConstants.NOMINAL_V;
      config.currentType = CurrentType.STATOR;
      config.maxCurrent = 50.0;
      config.canFreq.sensor = 1 / Chassis.odometryFrequency_Hz;
      config.canFreq.control = 0.02;
      config.canFreq.current = 0.02;
      config.canFreq.fault = 0.02;
      config.pidSlot = PIDSlot.SLOT0;
      config.pid0.p = 0.001;
      config.pid0.i = 0.0;
      config.pid0.d = 0.0;
      config.pid0.f = 0.001;
      config.pid0.allowableError = 0.0;
      config.pid0.iZone = 0.0;
      config.absoluteWrapping = false;
      config.inverted = false;
      config.encoderType = EncoderType.INTERNAL;
      config.maxOutput = 1.0;

      if (RobotBase.isReal()) {
        LogUtil.createTunablePID(RobotConstants.TUNING_PREFIX + name, realPID, tunable::get);
        LogUtil.createTunableFF(RobotConstants.TUNING_PREFIX + name, realFF, tunable::get);
      } else {
        LogUtil.createTunablePID(RobotConstants.TUNING_PREFIX + name, simPID, tunable::get);
        LogUtil.createTunableFF(RobotConstants.TUNING_PREFIX + name, simFF, tunable::get);
      }
    }
  }

  public static class Azimuth {
    public static final String name = DrivetrainConstants.NAME + "/Azimuth";

    // physical properties
    public static final double moi_kgm2 = 0.02;
    /** FL, FR, BL, BR */
    public static final int[] canIds = new int[] {8, 2, 6, 4};

    public static final double reduction = 25;
    public static final DCMotor gearbox = DCMotor.getNEO(1).withReduction(reduction);

    // software limits
    public static final ThriftyNovaConfig config = new ThriftyNovaConfig();

    public static final SimpleMotorFeedforward
        realFF = new SimpleMotorFeedforward(0.03, 0.04, 0.00, RobotConstants.CODE_PERIOD_s),
        simFF = new SimpleMotorFeedforward(0.004, 0.4960674, 0.006, RobotConstants.CODE_PERIOD_s);
    public static final PIDController
        realPID = new PIDController(0.4, 0.0, 0.0, RobotConstants.CODE_PERIOD_s),
        simPID = new PIDController(4, 0.0, 0.05, RobotConstants.CODE_PERIOD_s);

    static {
      config.brakeMode = true;
      config.voltageCompensation = null;
      config.currentType = CurrentType.STATOR;
      config.maxCurrent = 30.0;
      config.canFreq.sensor = 1 / Chassis.odometryFrequency_Hz;
      config.canFreq.control = 0.02;
      config.canFreq.current = 0.02;
      config.canFreq.fault = 0.02;
      config.pidSlot = PIDSlot.SLOT0;
      config.pid0.p = .005;
      config.pid0.i = 0.0;
      config.pid0.d = 0.0;
      config.pid0.f = 0.0;
      config.pid0.allowableError = 0.0;
      config.pid0.iZone = 0.0;
      config.absoluteWrapping = true;
      config.externalEncoder = ExternalEncoder.THRIFTY_10_PIN_ENCODER;
      config.inverted = true;
      config.encoderType = EncoderType.ABS;

      realPID.enableContinuousInput(0, 2 * Math.PI);
      simPID.enableContinuousInput(0, 2 * Math.PI);

      if (RobotBase.isReal()) {
        LogUtil.createTunablePID(RobotConstants.TUNING_PREFIX + name, realPID, tunable::get);
        LogUtil.createTunableFF(RobotConstants.TUNING_PREFIX + name, realFF, tunable::get);
      } else {
        LogUtil.createTunablePID(RobotConstants.TUNING_PREFIX + name, simPID, tunable::get);
        LogUtil.createTunableFF(RobotConstants.TUNING_PREFIX + name, simFF, tunable::get);
      }
    }
  }

  public static class AbsEncoder {
    public static final String name = DrivetrainConstants.NAME + "/AbsEncoder";

    // /** FL, FR, BL, BR */
    // public static final int[] analogPorts = new int[] {0, 1, 2, 3};

    // /** FL, FR, BL, BR. Rotation of each absolute encoder when the wheels face forward */
    // public static final double[] zeroRotations_rad =
    //     new double[] {4.26675, 2.99095, 2.40695, 3.355259};

    /** FL, FR, BL, BR. Rotation of each absolute encoder when the wheels face forward */
    public static final int[] zeroRotations_ticks =
        new int[] {1837 + 2048, 2565 - 2048, 2273 - 2048, 1339 + 2048};
  }
}
