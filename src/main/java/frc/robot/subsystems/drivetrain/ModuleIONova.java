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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel; // .MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.AbsEncoder;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Azimuth;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Drive;
import frc.robot.util.UnitUtil;
import java.util.Queue;

/**
 * Module IO implementation for Thrifty Nova drive motor controller, Thrifty Nova azimuth motor
 * controller, and Thrifty absolute encoder.
 */
public class ModuleIONova implements ModuleIO {
  // Hardware objects
  private final ThriftyNova driveNova;
  private final SparkMax driveSparkMax;
  private final ThriftyNova azimuthNova;
  private final SparkMax azimuthSparkMax;

  private final AnalogEncoder analogEncoder;

  private final int moduleIndex;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> azimuthPositionQueue;

  private double lastNextDriveVelocity_radPs = 0.0;
  private double currentDriveVelocity_radPs = 0.0;
  private double lastNextAzimuthVelocity_radPs = 0.0;
  private double currentAzimuthPosition_rad = 0.0;

  private double driveCommandedVoltage = 0.0;
  private double azimuthCommandedVoltage = 0.0;

  public ModuleIONova(int module) {
    moduleIndex = module;
    // azimuthNova = new ThriftyNova(Azimuth.canIds[module], MotorType.NEO);
    if ((module == 0) || (module == 1)) { // FL, FR are still novas
      driveNova = new ThriftyNova(Drive.canIds[module], MotorType.NEO);
      driveSparkMax = null;
    } else {
      driveNova = null;
      driveSparkMax = new SparkMax(Drive.canIds[module], SparkLowLevel.MotorType.kBrushless);
    }

    if (module == 2) { // BL Azimuth is also a sparkmax

      azimuthNova = null;

      azimuthSparkMax = new SparkMax(Azimuth.canIds[module], SparkLowLevel.MotorType.kBrushless);
      azimuthSparkMax.configure(
          Azimuth.coastAzimuthConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);

      analogEncoder = new AnalogEncoder(3, 1.0, (3.68 - 1.08) / (2 * Math.PI));
      analogEncoder.setInverted(true);

    } else {
      analogEncoder = null;
      azimuthSparkMax = null;
      azimuthNova = new ThriftyNova(Azimuth.canIds[module], MotorType.NEO);
    }

    // Configure drive motor
    System.out.println(
        "Configuring drive motor. Module: " + module + "  CAN Id: " + Drive.canIds[module]);
    if (driveSparkMax == null) {
      driveNova.applyConfig(Drive.config);
    } else {

      driveSparkMax.configure(
          Drive.sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    System.out.println("Finished configuring drive motor. Module: " + module + "  CAN Id: "
        + Drive.canIds[module]);

    // Configure azimuth motor
    System.out.println(
        "Configuring Azimuth motor. Module: " + module + "  CAN Id: " + Azimuth.canIds[module]);
    Azimuth.config.absOffset = AbsEncoder.zeroRotations_ticks[module];
    if (azimuthSparkMax == null) {
      azimuthNova.applyConfig(Azimuth.config);
    } else {
      azimuthSparkMax.configure(
          Azimuth.sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    System.out.println("Finished configuring Azimuth motor. Module: " + module + "  CAN Id: "
        + Azimuth.canIds[module]);

    // Create odometry queues
    timestampQueue = NovaOdometryThread.getInstance().makeTimestampQueue();
    if (driveSparkMax == null) {
      drivePositionQueue = NovaOdometryThread.getInstance()
          .registerSignal(
              () -> UnitUtil.rotTorad(driveNova.getPositionInternal() / Drive.reduction));
    } else {
      drivePositionQueue = NovaOdometryThread.getInstance()
          .registerSignal(
              () -> UnitUtil.rotTorad(driveSparkMax.getEncoder().getPosition() / Drive.reduction));
    }

    if (azimuthSparkMax == null) {
      azimuthPositionQueue =
          NovaOdometryThread.getInstance().registerSignal(() -> -azimuthNova.getPositionAbs());
    } else {
      azimuthPositionQueue = NovaOdometryThread.getInstance()
          .registerSignal(() -> -azimuthSparkMax.getAbsoluteEncoder().getPosition());
    }
    // new TunableDouble(
    //     "Tuning/Drive/1 P", Drive.config.pid0.p, () -> true, p -> driveNova.pid0.setP(p));
    // new TunableDouble(
    //     "Tuning/Drive/2 I", Drive.config.pid0.i, () -> true, i -> driveNova.pid0.setI(i));
    // new TunableDouble(
    //     "Tuning/Drive/3 D", Drive.config.pid0.d, () -> true, d -> driveNova.pid0.setD(d));
    // new TunableDouble(
    //     "Tuning/Drive/4 F", Drive.config.pid0.f, () -> true, f -> driveNova.pid0.setFF(f));
    // new TunableDouble(
    //     "Tuning/Drive/5 allowableError",
    //     Drive.config.pid0.allowableError,
    //     () -> true,
    //     error -> driveNova.pid0.setAllowableError(error));
    // new TunableDouble(
    //     "Tuning/Drive/6 accumulatorCap",
    //     Drive.config.pid0.iZone,
    //     () -> true,
    //     cap -> driveNova.pid0.setAccumulatorCap(cap));
    // new TunableDouble(
    //     "Tuning/Azimuth/1 P", Azimuth.config.pid0.p, () -> true, p -> azimuthNova.pid0.setP(p));
    // new TunableDouble(
    //     "Tuning/Azimuth/2 I", Azimuth.config.pid0.i, () -> true, i -> azimuthNova.pid0.setI(i));
    // new TunableDouble(
    //     "Tuning/Azimuth/3 D", Azimuth.config.pid0.d, () -> true, d -> azimuthNova.pid0.setD(d));
    // new TunableDouble(
    //     "Tuning/Azimuth/4 F", Azimuth.config.pid0.f, () -> true, f -> azimuthNova.pid0.setFF(f));
    // new TunableDouble(
    //     "Tuning/Azimuth/5 allowableError",
    //     Azimuth.config.pid0.allowableError,
    //     () -> true,
    //     error -> azimuthNova.pid0.setAllowableError(error));
    // new TunableDouble(
    //     "Tuning/Azimuth/6 accumulatorCap",
    //     Azimuth.config.pid0.iZone,
    //     () -> true,
    //     cap -> azimuthNova.pid0.setAccumulatorCap(cap));
    if (azimuthSparkMax == null) {
      currentAzimuthPosition_rad = UnitUtil.rotTorad(1 - azimuthNova.getPositionAbs());
    } else {
      currentAzimuthPosition_rad =
          UnitUtil.rotTorad(1 - azimuthSparkMax.getAbsoluteEncoder().getPosition());
    }
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    if (driveSparkMax == null) {
      inputs.drive.position.mut_replace(
          UnitUtil.rotTorad(driveNova.getPositionInternal() / Drive.reduction), Radians);
      inputs.drive.velocity.mut_replace(
          UnitUtil.rotTorad(driveNova.getVelocityInternal() / Drive.reduction), RadiansPerSecond);
      inputs.drive.outputVoltage.mut_replace(driveNova.getAppliedVoltage(), Volts);
      inputs.drive.inputVoltage.mut_replace(driveNova.getVoltage(), Volts);
      inputs.drive.outputCurrent.mut_replace(driveNova.getStatorCurrent(), Amps);
      inputs.drive.inputCurrent.mut_replace(driveNova.getSupplyCurrent(), Amps);
      inputs.drive.errors = driveNova.errors.toArray(ThriftyNova.Error[]::new);
      inputs.drive.connected = true;
    } else {
      inputs.drive.position.mut_replace(
          UnitUtil.rotTorad(driveSparkMax.getEncoder().getPosition() / Drive.reduction), Radians);
      inputs.drive.velocity.mut_replace(
          UnitUtil.rotTorad(driveSparkMax.getEncoder().getVelocity() / Drive.reduction / 60),
          RadiansPerSecond);
      inputs.drive.outputVoltage.mut_replace(
          driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage(), Volts);
      inputs.drive.inputVoltage.mut_replace(driveSparkMax.getBusVoltage(), Volts);
      inputs.drive.outputCurrent.mut_replace(driveSparkMax.getOutputCurrent(), Amps);
      inputs.drive.inputCurrent.mut_replace(
          Math.abs(driveSparkMax.getOutputCurrent() * driveSparkMax.getAppliedOutput()), Amps);
      // inputs.drive.errors = driveSparkMax.errors.toArray(ThriftyNova.Error[]::new);
      inputs.drive.connected = true;
    }
    currentDriveVelocity_radPs = inputs.drive.velocity.in(RadiansPerSecond);

    // Update azimuth inputs
    if (azimuthSparkMax == null) { // normal
      inputs.absoluteEncoder.heading = Rotation2d.fromRotations(1 - azimuthNova.getPositionAbs());
      currentAzimuthPosition_rad = UnitUtil.rotTorad(1 - azimuthNova.getPositionAbs());
      inputs.absoluteEncoder.connected = true;

      inputs.azimuth.position.mut_replace(
          UnitUtil.rotTorad(azimuthNova.getPositionInternal() / Azimuth.reduction), Radians);
      inputs.azimuth.velocity.mut_replace(
          UnitUtil.rotTorad(azimuthNova.getVelocityInternal() / Azimuth.reduction),
          RadiansPerSecond);
      inputs.azimuth.outputVoltage.mut_replace(azimuthNova.getAppliedVoltage(), Volts);
      inputs.azimuth.inputVoltage.mut_replace(azimuthNova.getVoltage(), Volts);
      inputs.azimuth.outputCurrent.mut_replace(azimuthNova.getStatorCurrent(), Amps);
      inputs.azimuth.inputCurrent.mut_replace(azimuthNova.getSupplyCurrent(), Amps);
      inputs.azimuth.errors = azimuthNova.errors.toArray(ThriftyNova.Error[]::new);
      inputs.azimuth.connected = true;
    } else { // analog encoder for BL azimuth
      inputs.absoluteEncoder.heading = Rotation2d.fromRotations(1 - analogEncoder.get());
      currentAzimuthPosition_rad = UnitUtil.rotTorad(1 - analogEncoder.get());
      inputs.absoluteEncoder.connected = true;

      inputs.azimuth.position.mut_replace(
          UnitUtil.rotTorad(azimuthSparkMax.getEncoder().getPosition() / Azimuth.reduction),
          Radians);
      inputs.azimuth.velocity.mut_replace(
          UnitUtil.rotTorad(azimuthSparkMax.getEncoder().getVelocity() / Azimuth.reduction),
          RadiansPerSecond);
      inputs.azimuth.outputVoltage.mut_replace(
          azimuthSparkMax.getAppliedOutput() * azimuthSparkMax.getBusVoltage(), Volts);
      inputs.azimuth.inputVoltage.mut_replace(azimuthSparkMax.getBusVoltage(), Volts);
      inputs.azimuth.outputCurrent.mut_replace(azimuthSparkMax.getOutputCurrent(), Amps);
      inputs.azimuth.inputCurrent.mut_replace(
          Math.abs(azimuthSparkMax.getOutputCurrent() * azimuthSparkMax.getAppliedOutput()), Amps);
      // inputs.azimuth.errors = azimuthSparkMax.errors.toArray(ThriftyNova.Error[]::new);
      inputs.azimuth.connected = true;
    }
    // Update odometry inputs
    inputs.odometry.timestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometry.drivePositions_rad = drivePositionQueue.stream()
        .mapToDouble((Double value) -> UnitUtil.rotTorad(value / Drive.reduction))
        .toArray();
    inputs.odometry.azimuthPositions_rad = azimuthPositionQueue.stream()
        .mapToDouble((Double value) -> UnitUtil.rotTorad(value)) // - zeroRotation_rad)
        .toArray();

    timestampQueue.clear();
    drivePositionQueue.clear();
    azimuthPositionQueue.clear();

    inputs.drive.commandedVoltage.mut_replace(driveCommandedVoltage, Volts);
    inputs.azimuth.commandedVoltage.mut_replace(azimuthCommandedVoltage, Volts);
  }

  @Override
  public void setDriveVoltage(double voltage_V) {
    // TODO: make sane conversion method for this
    if (driveNova == null) {
      driveSparkMax.setVoltage(voltage_V * 12.0);
    } else {
      driveNova.setVoltage(voltage_V);
    }
    driveCommandedVoltage = voltage_V;
  }

  @Override
  public void setAzimuthVoltage(double voltage_V) {
    if (azimuthNova == null) {
      azimuthSparkMax.setVoltage(voltage_V * 12.0);
    } else {
      azimuthNova.setVoltage(voltage_V);
    }
    azimuthCommandedVoltage = voltage_V;
  }

  @Override
  public void setNextDriveState(double nextVelocity_radPs, double nextAcceleration_radPs2) {
    // driveNova.setVelocityInternal(
    //     nextVelocity_radPs * Drive.reduction / (2 * Math.PI),
    //     Drive.realFF.calculate(nextVelocity_radPs, nextAcceleration_radPs2));
    double driveCommandedVoltage =
        (Drive.realPID.calculate(currentDriveVelocity_radPs, lastNextDriveVelocity_radPs)
            + Drive.realFF.calculate(nextVelocity_radPs, nextAcceleration_radPs2));
    lastNextDriveVelocity_radPs = nextVelocity_radPs;
    if (driveSparkMax == null) {
      driveNova.setVoltage(driveCommandedVoltage);
    } else {
      driveSparkMax.setVoltage(driveCommandedVoltage * 12.0);
    }
  }

  @Override
  public void setNextAzimuthState(double nextPosition_rad, double nextVelocity_radPs) {
    // azimuthNova.setPositionAbs(UnitUtil.radTorot(-nextPosition_rad), .1);
    azimuthCommandedVoltage = -Azimuth.realPID.calculate(
            currentAzimuthPosition_rad, nextPosition_rad)
        - Azimuth.realFF.calculateWithVelocities(lastNextAzimuthVelocity_radPs, nextVelocity_radPs);
    if (moduleIndex == 2) {
      azimuthSparkMax.setVoltage(0);
    } else {
      if (azimuthSparkMax == null) {
        azimuthNova.setVoltage(azimuthCommandedVoltage);
        // Logger.recordOutput("AzimuthFeedforward", Drive.realFF.getKs());
        lastNextAzimuthVelocity_radPs = nextVelocity_radPs;
      } else {
        azimuthSparkMax.setVoltage(azimuthCommandedVoltage * 12);
        // Logger.recordOutput("AzimuthFeedforward", Drive.realFF.getKs());
        lastNextAzimuthVelocity_radPs = nextVelocity_radPs;
      }
    }
  }

  @Override
  public void setCoastMode(boolean coast) {
    if (driveSparkMax == null) {
      driveNova.setBrakeMode(!coast);
    } else {
      SparkMaxConfig coastMode = new SparkMaxConfig();
      if (coast) {
        coastMode.idleMode(IdleMode.kCoast);
      } else {
        coastMode.idleMode(IdleMode.kBrake);
      }
      driveSparkMax.configure(
          coastMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }
}

