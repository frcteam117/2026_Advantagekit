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

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.drive.DriveConstants.DriveMotor;
import frc.robot.subsystems.drive.DriveConstants.TurnMotor;
import frc.robot.util.SparkUtil;
import java.util.Arrays;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor, turnMotor;
  private final PIDController drivePID = DriveMotor.simPID, turnPID = TurnMotor.simPID;
  private boolean driveClosedLoop = false, turnClosedLoop = false;
  private double lastNextDriveVelocity_radPs = 0.0,
      currentTurnVelocity_radPs = 0.0,
      lastNextTurnPosition_rad = 0.0,
      driveAppliedVolts = 0.0,
      turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    this.driveMotor = moduleSimulation
        .useGenericMotorControllerForDrive()
        .withCurrentLimit(Amps.of(DriveMotor.config.limits.getMaxStatorCurrent()));
    this.turnMotor = moduleSimulation
        .useGenericControllerForSteer()
        .withCurrentLimit(Amps.of(TurnMotor.config.limits.getMaxSupplyCurrent()));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (!driveClosedLoop) {
      DriveMotor.simPID.reset();
    }
    if (!turnClosedLoop) {
      TurnMotor.simPID.reset();
    }

    // Update simulation state
    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePosition_rad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocity_radPs = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveVoltage_V = driveAppliedVolts;
    inputs.driveStatorCurrent_A =
        Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));
    inputs.driveSupplyCurrent_A =
        Math.abs(moduleSimulation.getDriveMotorSupplyCurrent().in(Amps));

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition_rad =
        moduleSimulation.getSteerRelativeEncoderPosition().in(Radians) / TurnMotor.reduction;
    inputs.turnAbsolutePosition_rad = moduleSimulation.getSteerAbsoluteFacing().getRadians();
    inputs.turnVelocity_radPs =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    currentTurnVelocity_radPs = inputs.turnVelocity_radPs;
    inputs.turnVoltage_V = turnAppliedVolts;
    inputs.turnStatorCurrent_A =
        Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));
    inputs.turnSupplyCurrent_A =
        Math.abs(moduleSimulation.getSteerMotorSupplyCurrent().in(Amps));

    // Update odometry inputs
    inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositions_rad = Arrays.stream(
            moduleSimulation.getCachedDriveWheelFinalPositions())
        .mapToDouble(angle -> angle.in(Radians))
        .toArray();
    inputs.odometryTurnPositions_rad = Arrays.stream(
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
  public void setTurnVoltage(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
    lastNextTurnPosition_rad = moduleSimulation.getSteerAbsoluteFacing().getRadians();
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
    driveAppliedVolts = DriveMotor.simFF.calculate(nextVelocity_radPs, nextAcceleration_radPs2)
        + DriveMotor.simPID.calculate(
            moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond),
            lastNextDriveVelocity_radPs);
    lastNextDriveVelocity_radPs = nextVelocity_radPs;
  }

  @Override
  public void setNextTurnState(double nextPosition_rad, double nextVelocity_radPs) {
    turnClosedLoop = true;
    turnAppliedVolts =
        TurnMotor.simFF.calculateWithVelocities(currentTurnVelocity_radPs, nextVelocity_radPs)
            + TurnMotor.simPID.calculate(
                moduleSimulation.getSteerAbsoluteFacing().getRadians(), lastNextTurnPosition_rad);
    lastNextTurnPosition_rad = nextPosition_rad;
  }
}
