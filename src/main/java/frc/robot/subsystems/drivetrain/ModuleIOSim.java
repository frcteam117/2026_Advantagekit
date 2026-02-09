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

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Azimuth;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Drive;
import frc.robot.util.SparkUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor, azimuthMotor;
  private final PIDController drivePID = Drive.simPID, azimuthPID = Azimuth.simPID;
  private boolean driveClosedLoop = false, azimuthClosedLoop = false;
  private double lastNextDriveVelocity_radPs = 0.0,
      currentAzimuthVelocity_radPs = 0.0,
      lastNextAzimuthPosition_rad = 0.0,
      driveAppliedVolts = 0.0,
      azimuthAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    this.driveMotor = moduleSimulation
        .useGenericMotorControllerForDrive()
        .withCurrentLimit(Amps.of(Drive.config.maxCurrent));
    this.azimuthMotor = moduleSimulation
        .useGenericControllerForSteer()
        .withCurrentLimit(Amps.of(Azimuth.config.maxCurrent));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (!driveClosedLoop) {
      Drive.simPID.reset();
    }
    if (!azimuthClosedLoop) {
      Azimuth.simPID.reset();
    }

    // Update simulation state
    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    azimuthMotor.requestVoltage(Volts.of(azimuthAppliedVolts));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePosition_rad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocity_radPs = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveVoltage_V = driveAppliedVolts;
    inputs.driveStatorCurrent_A =
        Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));
    inputs.driveSupplyCurrent_A =
        Math.abs(moduleSimulation.getDriveMotorSupplyCurrent().in(Amps));

    // Update azimuth inputs
    inputs.azimuthConnected = true;
    inputs.azimuthPosition_rad =
        moduleSimulation.getSteerRelativeEncoderPosition().in(Radians) / Azimuth.reduction;
    inputs.azimuthAbsolutePosition_rad =
        moduleSimulation.getSteerAbsoluteFacing().getRadians();
    inputs.azimuthVelocity_radPs =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    currentAzimuthVelocity_radPs = inputs.azimuthVelocity_radPs;
    inputs.azimuthVoltage_V = azimuthAppliedVolts;
    inputs.azimuthStatorCurrent_A =
        Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));
    inputs.azimuthSupplyCurrent_A =
        Math.abs(moduleSimulation.getSteerMotorSupplyCurrent().in(Amps));

    // Update odometry inputs
    inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositions_rad = Arrays.stream(
            moduleSimulation.getCachedDriveWheelFinalPositions())
        .mapToDouble(angle -> angle.in(Radians))
        .toArray();
    inputs.odometryAzimuthPositions_rad = Arrays.stream(
            moduleSimulation.getCachedSteerAbsolutePositions())
        .mapToDouble((rotation) -> rotation.getRadians())
        .toArray();
  }

  @Override
  public void setDriveVoltage(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
    lastNextDriveVelocity_radPs = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
  }

  @Override
  public void setAzimuthVoltage(double output) {
    azimuthClosedLoop = false;
    azimuthAppliedVolts = output;
    lastNextAzimuthPosition_rad = moduleSimulation.getSteerAbsoluteFacing().getRadians();
  }

  // @Override
  // public void setNextDriveVelocity(double nextVelocity_radPs) {
  //   driveClosedLoop = true;
  //   driveFFVolts = DriveMotor.simFF.calculate(nextVelocity_radPs);
  //   this.nextVelocity_radPs = nextVelocity_radPs;
  // }

  @Override
  public void setNextDriveState(double nextVelocity_radPs, double nextAcceleration_radPs2) {
    driveClosedLoop = true;
    driveAppliedVolts = Drive.simFF.calculate(nextVelocity_radPs, nextAcceleration_radPs2)
        + Drive.simPID.calculate(
            moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond),
            lastNextDriveVelocity_radPs);
    lastNextDriveVelocity_radPs = nextVelocity_radPs;
  }

  @Override
  public void setNextAzimuthState(double nextPosition_rad, double nextVelocity_radPs) {
    azimuthClosedLoop = true;
    azimuthAppliedVolts = Azimuth.simFF.calculateWithVelocities(
            currentAzimuthVelocity_radPs, nextVelocity_radPs)
        + Azimuth.simPID.calculate(
            moduleSimulation.getSteerAbsoluteFacing().getRadians(), lastNextAzimuthPosition_rad);
    lastNextAzimuthPosition_rad = nextPosition_rad;
  }
}
