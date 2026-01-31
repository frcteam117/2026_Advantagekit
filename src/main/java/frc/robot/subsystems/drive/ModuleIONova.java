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

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.MotorType;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.subsystems.drive.DriveConstants.AbsEncoder;
import frc.robot.subsystems.drive.DriveConstants.Azimuth;
import frc.robot.subsystems.drive.DriveConstants.Wheel;
import frc.robot.util.UnitUtil;
import java.util.Queue;

/**
 * Module IO implementation for Thrifty Nova drive motor controller, Thrifty Nova turn motor
 * controller, and Thrifty absolute encoder.
 */
public class ModuleIONova implements ModuleIO {
  private final double zeroRotation_rad;

  // Motion profiling
  private double currentDriveVelocity_radPs = 0.0;
  private double lastNextDriveVelocity_radPs = 0.0;

  private double turnPosition_rad = 0.0;
  private double currentTurnVelocity_radPs = 0.0;

  // Hardware objects
  private final ThriftyNova driveNova;
  private final ThriftyNova turnNova;
  private final AnalogEncoder turnEncoder;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  public ModuleIONova(int module) {
    zeroRotation_rad = AbsEncoder.zeroRotations_rad[module];
    driveNova = new ThriftyNova(Wheel.canIds[module], MotorType.NEO);
    turnNova = new ThriftyNova(Azimuth.canIds[module], MotorType.NEO);
    turnEncoder = new AnalogEncoder(AbsEncoder.analogPorts[module]);

    // Configure drive motor
    System.out.println(
        "Configuring drive motor. Module: " + module + "  CAN Id: " + Wheel.canIds[module]);
    Wheel.config.configure(driveNova);
    System.out.println("Finished configuring drive motor. Module: " + module + "  CAN Id: "
        + Wheel.canIds[module]);

    // Configure turn motor
    System.out.println(
        "Configuring Turn motor. Module: " + module + "  CAN Id: " + Azimuth.canIds[module]);
    Azimuth.config.configure(turnNova);
    System.out.println("Finished configuring Turn motor. Module: " + module + "  CAN Id: "
        + Azimuth.canIds[module]);

    // Create odometry queues
    timestampQueue = NovaOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        NovaOdometryThread.getInstance().registerSignal(driveNova::getPositionInternal);
    turnPositionQueue = NovaOdometryThread.getInstance().registerSignal(turnEncoder::get);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    inputs.drivePosition_rad = UnitUtil.rotTorad(driveNova.getPositionInternal() / Wheel.reduction);
    inputs.driveVelocity_radPs =
        UnitUtil.RPMToradPs(driveNova.getVelocityInternal()) / Wheel.reduction;
    currentDriveVelocity_radPs = inputs.driveVelocity_radPs;
    inputs.driveVoltage_V = driveNova.getVoltage();
    inputs.driveStatorCurrent_A = driveNova.getStatorCurrent();
    inputs.driveSupplyCurrent_A = driveNova.getSupplyCurrent();
    // inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    // sparkStickyFault = false;
    inputs.turnAbsolutePosition_rad = UnitUtil.rotTorad(turnEncoder.get()) - zeroRotation_rad;
    turnPosition_rad = inputs.turnAbsolutePosition_rad;
    inputs.turnPosition_rad =
        UnitUtil.rotTorad(turnNova.getPositionInternal() / Azimuth.reduction) - zeroRotation_rad;
    inputs.turnVelocity_radPs =
        UnitUtil.RPMToradPs(turnNova.getVelocityInternal() / Azimuth.reduction);
    currentTurnVelocity_radPs = inputs.turnVelocity_radPs;
    inputs.turnVoltage_V = turnNova.getVoltage();
    inputs.turnStatorCurrent_A = turnNova.getStatorCurrent();
    inputs.turnSupplyCurrent_A = turnNova.getSupplyCurrent();
    // inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositions_rad = drivePositionQueue.stream()
        .mapToDouble((Double value) -> UnitUtil.rotTorad(value / Wheel.reduction))
        .toArray();
    inputs.odometryTurnPositions_rad = turnPositionQueue.stream()
        .mapToDouble((Double value) -> UnitUtil.rotTorad(value) - zeroRotation_rad)
        .toArray();
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double voltage_V) {
    driveNova.setVoltage(voltage_V);
  }

  @Override
  public void setTurnVoltage(double voltage_V) {
    turnNova.setVoltage(voltage_V);
  }

  @Override
  public void setNextDriveVelocity(double nextVelocity_radPs) {
    driveNova.setVoltage(
        Wheel.realFF.calculateWithVelocities(currentDriveVelocity_radPs, nextVelocity_radPs)
            + Wheel.realPID.calculate(currentDriveVelocity_radPs, lastNextDriveVelocity_radPs));
    lastNextDriveVelocity_radPs = nextVelocity_radPs;
  }

  @Override
  public void setNextDriveState(double nextVelocity_radPs, double nextAcceleration_radPs2) {
    driveNova.setVoltage(Wheel.realFF.calculate(nextVelocity_radPs, nextAcceleration_radPs2)
        + Wheel.realPID.calculate(currentDriveVelocity_radPs, lastNextDriveVelocity_radPs));
    lastNextDriveVelocity_radPs = nextVelocity_radPs;
  }

  @Override
  public void setNextTurnState(double nextPosition_rad, double nextVelocity_radPs) {
    turnNova.setVoltage(
        Azimuth.realFF.calculateWithVelocities(currentTurnVelocity_radPs, nextVelocity_radPs)
            + Azimuth.realPID.calculate(turnPosition_rad, nextPosition_rad));
  }
}
