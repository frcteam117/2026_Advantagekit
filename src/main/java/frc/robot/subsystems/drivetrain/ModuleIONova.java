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
import frc.robot.util.components.bases.ComponentStates.Motor_State;
import java.util.Queue;

/**
 * Module IO implementation for Thrifty Nova drive motor controller, Thrifty Nova azimuth motor
 * controller, and Thrifty absolute encoder.
 */
public class ModuleIONova implements ModuleIO {
  private final double zeroRotation_rad;

  // Motion profiling
  private double currentDriveVelocity_radPs = 0.0;

  private double currentAzimuthVelocity_radPs = 0.0;

  // Hardware objects
  private final ThriftyNova driveNova;
  private final ThriftyNova azimuthNova;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> azimuthPositionQueue;

  public ModuleIONova(int module) {
    zeroRotation_rad = AbsEncoder.zeroRotations_rad[module];
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
    azimuthNova.applyConfig(Azimuth.config);
    ;
    System.out.println("Finished configuring Azimuth motor. Module: " + module + "  CAN Id: "
        + Azimuth.canIds[module]);

    // Create odometry queues
    timestampQueue = NovaOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = NovaOdometryThread.getInstance()
        .registerSignal(() -> UnitUtil.rotTorad(driveNova.getPositionInternal() / Drive.reduction));
    azimuthPositionQueue =
        NovaOdometryThread.getInstance().registerSignal(() -> -azimuthNova.getPositionAbs());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    inputs.driveMotor_State = new Motor_State(
        UnitUtil.rotTorad(driveNova.getPositionInternal() / Drive.reduction),
        UnitUtil.RPMToradPs(driveNova.getVelocityInternal()) / Drive.reduction,
        driveNova.getVoltage(),
        driveNova.getStatorCurrent(),
        driveNova.getSupplyCurrent());
    // inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update azimuth inputs
    // sparkStickyFault = false;
    inputs.azimuthAbsolutePosition_rad =
        UnitUtil.rotTorad(-azimuthNova.getPositionAbs()); // - zeroRotation_rad;

    inputs.azimuthMotor_State = new Motor_State(
        UnitUtil.rotTorad(azimuthNova.getPositionInternal() / Azimuth.reduction) - zeroRotation_rad,
        UnitUtil.RPMToradPs(azimuthNova.getVelocityInternal() / Azimuth.reduction),
        azimuthNova.getVoltage(),
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

  @Override
  public void setNextDriveVelocity(double nextVelocity_radPs) {
    driveNova.setVelocityInternal(
        nextVelocity_radPs,
        Drive.realFF.calculateWithVelocities(currentDriveVelocity_radPs, nextVelocity_radPs));
  }

  @Override
  public void setNextDriveState(double nextVelocity_radPs, double nextAcceleration_radPs2) {
    driveNova.setVelocityInternal(
        nextVelocity_radPs, Drive.realFF.calculate(nextVelocity_radPs, nextAcceleration_radPs2));
  }

  @Override
  public void setNextAzimuthState(double nextPosition_rad, double nextVelocity_radPs) {
    azimuthNova.setPositionAbs(
        nextPosition_rad,
        Azimuth.realFF.calculateWithVelocities(currentAzimuthVelocity_radPs, nextVelocity_radPs));
  }
}
