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
import static frc.robot.Constants.nominalVoltage_V;
import static frc.robot.Constants.robotPeriod_s;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.logging.LogUtil;
import frc.robot.util.nova.NovaConfig;
import frc.robot.util.nova.NovaConfig.BrakeMode;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import swervelib.simulation.ironmaple.simulation.drivesims.COTS;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
  public static final String name = "Drive";
  public static final LoggedNetworkBoolean tunable =
      new LoggedNetworkBoolean("Tunable/" + name + "/.Tunable", false);
  public static final double wheelRadius_m = Units.inchesToMeters((3.875 - .12) / 2);
  public static final double maxSpeed_mPs = Units.feetToMeters(15);
  public static final double odometryFrequency_Hz = 100.0;
  public static final double trackWidth_m = Units.inchesToMeters(21.625 - (2 * 1.6875));
  public static final double wheelBase_m = Units.inchesToMeters(21.625 - (2 * 1.6875));
  public static final double driveBaseRadius_m = Math.hypot(trackWidth_m / 2.0, wheelBase_m / 2.0);
  public static final Translation2d[] moduleTranslations = new Translation2d[] {
    new Translation2d(trackWidth_m / 2.0, wheelBase_m / 2.0),
    new Translation2d(trackWidth_m / 2.0, -wheelBase_m / 2.0),
    new Translation2d(-trackWidth_m / 2.0, wheelBase_m / 2.0),
    new Translation2d(-trackWidth_m / 2.0, -wheelBase_m / 2.0)
  };

  public static class DriveMotor {
    /** FL, FR, BL, BR */
    public static final int[] canIds = new int[] {3, 5, 1, 7};

    public static final double reduction = 6.25666667;
    public static final DCMotor gearbox = DCMotor.getNeoVortex(1);
    public static final NovaConfig config = new NovaConfig();

    // Drive PID configuration
    public static final SimpleMotorFeedforward
        realFF = new SimpleMotorFeedforward(0.013, 0.01, 0.0, robotPeriod_s),
        simFF = new SimpleMotorFeedforward(0.036968, 0.15869, 0.034, robotPeriod_s);
    public static final PIDController realPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s),
        simPID = new PIDController(0.23931, 0.0, 0.0, robotPeriod_s);

    static {
      config.setBrakeMode(BrakeMode.BRAKE).setVoltageCompensation(nominalVoltage_V);
      config.limits.setMaxStatorCurrent(60).setMaxSupplyCurrent(60);
      config
          .canFreq
          .setSensorPeriod(1 / odometryFrequency_Hz)
          .setControlPeriod(0.02)
          .setCurrentPeriod(0.02)
          .setFaultPeriod(0.02);

      LogUtil.createTunablePID("Tunable/" + name + "/DriveMotor/real", realPID, tunable::get);
      LogUtil.createTunableFF("Tunable/" + name + "/DriveMotor/real", realFF, tunable::get);
      LogUtil.createTunablePID("Tunable/" + name + "/DriveMotor/sim", simPID, tunable::get);
      LogUtil.createTunableFF("Tunable/" + name + "/DriveMotor/sim", simFF, tunable::get);
    }
  }

  public static class TurnMotor {
    // Turn motor configuration
    /** FL, FR, BL, BR */
    public static final int[] canIds = new int[] {4, 6, 2, 8};

    public static final double reduction = 25;
    public static final DCMotor gearbox = DCMotor.getNEO(1);
    public static final NovaConfig config = new NovaConfig();

    /** PID controllers for the turn (azimuth) motor in each swerve module. */
    public static final SimpleMotorFeedforward
        realFF = new SimpleMotorFeedforward(0.01, 0.051, 0.0, robotPeriod_s),
        simFF = new SimpleMotorFeedforward(0.004, 0.4960674, 0.006, robotPeriod_s);

    public static final PIDController realPID = new PIDController(0.5, 0.0, 0.0, robotPeriod_s),
        simPID = new PIDController(4, 0.0, 0.05, robotPeriod_s);

    static {
      config
          .setInversion(false)
          .setBrakeMode(BrakeMode.BRAKE)
          .setVoltageCompensation(nominalVoltage_V);
      config.limits.setMaxStatorCurrent(50).setMaxSupplyCurrent(20);
      config
          .canFreq
          .setSensorPeriod(1 / odometryFrequency_Hz)
          .setControlPeriod(0.02)
          .setCurrentPeriod(0.02)
          .setFaultPeriod(0.02);

      realPID.enableContinuousInput(0, 2 * Math.PI);
      simPID.enableContinuousInput(0, 2 * Math.PI);
      LogUtil.createTunablePID("Tunable/" + name + "/TurnMotor/real", realPID, tunable::get);
      LogUtil.createTunableFF("Tunable/" + name + "/TurnMotor/real", realFF, tunable::get);
      LogUtil.createTunablePID("Tunable/" + name + "/TurnMotor/sim", simPID, tunable::get);
      LogUtil.createTunableFF("Tunable/" + name + "/TurnMotor/sim", simFF, tunable::get);
    }
  }

  public static class AbsEncoder {
    /** FL, FR, BL, BR */
    public static final int[] analogPorts = new int[] {0, 1, 2, 3};

    // Zeroed rotation values for each module
    /** FL, FR, BL, BR */
    public static final double[] zeroRotations_rad =
        new double[] {4.26675, 2.99095, 2.40695, 3.355259};
  }

  // PathPlanner configuration
  public static final double robotMass_kg = 18.35;
  public static final double robotMOI_kgm2 = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig = new RobotConfig(
      robotMass_kg,
      robotMOI_kgm2,
      new ModuleConfig(
          wheelRadius_m,
          maxSpeed_mPs,
          wheelCOF,
          DriveMotor.gearbox.withReduction(DriveMotor.reduction),
          DriveMotor.config.limits.getMaxStatorCurrent(),
          1),
      moduleTranslations);

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withBumperSize(Inches.of(30.625), Inches.of(30.625))
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(Kilogram.of(robotMass_kg))
          .withGyro(COTS.ofNav2X())
          .withSwerveModule(new SwerveModuleSimulationConfig(
              DriveMotor.gearbox,
              TurnMotor.gearbox,
              DriveMotor.reduction,
              TurnMotor.reduction,
              Volts.of(0.1),
              Volts.of(0.1),
              Meters.of(wheelRadius_m),
              KilogramSquareMeters.of(0.02),
              wheelCOF));
}
