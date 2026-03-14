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

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.MotorType;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.AbsEncoder;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Azimuth;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Drive;
import frc.robot.util.UnitUtil;
import frc.robot.util.components.bases.ComponentStates.AbsoluteEncoder_State;
import frc.robot.util.components.bases.ComponentStates.Motor_State;
import java.util.Queue;

/**
 * Module IO implementation for Thrifty Nova drive motor controller, Thrifty Nova azimuth motor
 * controller, and Thrifty absolute encoder.
 */
public class ModuleIONova implements ModuleIO {
  // Hardware objects
  private final ThriftyNova driveNova;
  private final ThriftyNova azimuthNova;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> azimuthPositionQueue;

  private double lastNextDriveVelocity_radPs = 0.0;
  private double lastNextAzimuthVelocity_radPs = 0.0;
  private double currentAzimuthPosition_rad = 0.0;

  public ModuleIONova(int module) {
    driveNova = new ThriftyNova(Drive.canIds[module], MotorType.NEO);
    azimuthNova = new ThriftyNova(Azimuth.canIds[module], MotorType.NEO);

    // Configure drive motor
    System.out.println(
        "Configuring drive motor. Module: " + module + "  CAN Id: " + Drive.canIds[module]);
    driveNova.applyConfig(Drive.config);
    System.out.println("Finished configuring drive motor. Module: " + module + "  CAN Id: "
        + Drive.canIds[module]);

    // Configure azimuth motor
    System.out.println(
        "Configuring Azimuth motor. Module: " + module + "  CAN Id: " + Azimuth.canIds[module]);
    Azimuth.config.absOffset = AbsEncoder.zeroRotations_ticks[module];
    azimuthNova.applyConfig(Azimuth.config);
    System.out.println("Finished configuring Azimuth motor. Module: " + module + "  CAN Id: "
        + Azimuth.canIds[module]);

    // Create odometry queues
    timestampQueue = NovaOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = NovaOdometryThread.getInstance()
        .registerSignal(() -> UnitUtil.rotTorad(driveNova.getPositionInternal() / Drive.reduction));
    azimuthPositionQueue =
        NovaOdometryThread.getInstance().registerSignal(() -> -azimuthNova.getPositionAbs());
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
    currentAzimuthPosition_rad = UnitUtil.rotTorad(1 - azimuthNova.getPositionAbs());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    inputs.driveMotor_State = new Motor_State(
        UnitUtil.rotTorad(driveNova.getPositionInternal() / Drive.reduction),
        UnitUtil.rotTorad(driveNova.getVelocityInternal() / Drive.reduction),
        driveNova.getAppliedVoltage(),
        driveNova.getStatorCurrent(),
        driveNova.getSupplyCurrent());
    // inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update azimuth inputs
    // sparkStickyFault = false;
    inputs.azimuthAbsolutePosition_State = new AbsoluteEncoder_State(
        UnitUtil.rotTorad(1 - azimuthNova.getPositionAbs())); // - zeroRotation_rad;
    currentAzimuthPosition_rad = UnitUtil.rotTorad(1 - azimuthNova.getPositionAbs());

    inputs.azimuthMotor_State = new Motor_State(
        UnitUtil.rotTorad(
            azimuthNova.getPositionInternal() / Azimuth.reduction), // - zeroRotation_rad,
        UnitUtil.rotTorad(azimuthNova.getVelocityInternal() / Azimuth.reduction),
        azimuthNova.getAppliedVoltage(),
        azimuthNova.getStatorCurrent(),
        azimuthNova.getSupplyCurrent());
    // inputs.azimuthConnected = azimuthConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositions_rad = drivePositionQueue.stream()
        .mapToDouble((Double value) -> UnitUtil.rotTorad(value / Drive.reduction))
        .toArray();
    inputs.odometryAzimuthPositions_rad = azimuthPositionQueue.stream()
        .mapToDouble((Double value) -> UnitUtil.rotTorad(value)) // - zeroRotation_rad)
        .toArray();
    timestampQueue.clear();
    drivePositionQueue.clear();
    azimuthPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double voltage_V) {
    driveNova.setVoltage(voltage_V);
  }

  @Override
  public void setAzimuthVoltage(double voltage_V) {
    azimuthNova.setVoltage(voltage_V);
  }

  //   @Override
  //   public void setNextDriveVelocity(double nextVelocity_radPs) {
  //     driveNova.setVelocityInternal(nextVelocity_radPs * Drive.reduction / 60);
  //     // Drive.realFF.calculateWithVelocities(currentDriveVelocity_radPs, nextVelocity_radPs));
  //   }

  @Override
  public void setNextDriveState(double nextVelocity_radPs, double nextAcceleration_radPs2) {
    // driveNova.setVelocityInternal(
    //     nextVelocity_radPs * Drive.reduction / (2 * Math.PI),
    //     Drive.realFF.calculate(nextVelocity_radPs, nextAcceleration_radPs2));
    driveNova.setVoltage(Drive.realPID.calculate(nextVelocity_radPs, lastNextDriveVelocity_radPs)
        + Drive.realFF.calculate(nextVelocity_radPs, nextAcceleration_radPs2));
    lastNextDriveVelocity_radPs = nextVelocity_radPs;
  }

  @Override
  public void setNextAzimuthState(double nextPosition_rad, double nextVelocity_radPs) {
    // azimuthNova.setPositionAbs(UnitUtil.radTorot(-nextPosition_rad), .1);
    azimuthNova.setVoltage(-Azimuth.realPID.calculate(currentAzimuthPosition_rad, nextPosition_rad)
        - Azimuth.realFF.calculateWithVelocities(
            lastNextAzimuthVelocity_radPs, nextVelocity_radPs));
    // Logger.recordOutput("AzimuthFeedforward", Drive.realFF.getKs());
    lastNextAzimuthVelocity_radPs = nextVelocity_radPs;
  }
}
