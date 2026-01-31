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

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    // public LinearMechanismState wheel;
    public double drivePosition_rad = 0.0;
    public double driveVelocity_radPs = 0.0;
    public double driveVoltage_V = 0.0;
    public double driveStatorCurrent_A = 0.0;
    public double driveSupplyCurrent_A = 0.0;

    public double turnAbsolutePosition_rad = 0.0;

    public boolean turnConnected = false;
    public double turnPosition_rad = 0.0;
    public double turnVelocity_radPs = 0.0;
    public double turnVoltage_V = 0.0;
    public double turnStatorCurrent_A = 0.0;
    public double turnSupplyCurrent_A = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositions_rad = new double[] {};
    public double[] odometryTurnPositions_rad = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Runs the drive motor at the specified voltage without feedback. */
  public default void setDriveVoltage(double voltage_V) {}

  /** Runs the turn motor at the specified voltage without feedback. */
  public default void setTurnVoltage(double voltage_V) {}

  /** Runs the drive motor at the next pidf voltage based on the given velocity. */
  public default void setNextDriveVelocity(double nextVelocity_radPs) {}

  /** Runs the drive motor at the next pidf voltage based on the given velocity and acceleration. */
  public default void setNextDriveState(
      double nextVelocity_radPs, double nextAcceleration_radPs2) {}

  /** Runs the turn motor at the next pidf voltage based on the given position and velocity. */
  public default void setNextTurnState(double position_rad, double velocity_radPs) {}
}
