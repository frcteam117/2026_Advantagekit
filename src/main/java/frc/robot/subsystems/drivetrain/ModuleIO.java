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

import frc.robot.util.components.bases.ComponentStates.AbsoluteEncoder_State;
import frc.robot.util.components.bases.ComponentStates.Motor_State;
import frc.robot.util.states.LoggableStateInputs;
import frc.robot.util.states.State;

public interface ModuleIO {
  public static class ModuleIOInputs implements LoggableStateInputs {
    public Motor_State driveMotor_State = new Motor_State(0, 0, 0, 0, 0);
    public AbsoluteEncoder_State azimuthAbsolutePosition_State = new AbsoluteEncoder_State(0);
    public Motor_State azimuthMotor_State = new Motor_State(0, 0, 0, 0, 0);

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositions_rad = new double[] {};
    public double[] odometryAzimuthPositions_rad = new double[] {};

    @Override
    public State[] getStates() {
      return new State[] {driveMotor_State, azimuthAbsolutePosition_State, azimuthMotor_State};
    }

    @Override
    public void setStates(State[] states) {
      driveMotor_State = (Motor_State) states[0];
      azimuthMotor_State = (Motor_State) states[1];
    }

    @Override
    public String[] getStateNames() {
      return new String[] {"Drive", "Absolute", "Azimuth"};
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Runs the drive motor at the specified voltage without feedback. */
  public default void setDriveVoltage(double voltage_V) {}

  /** Runs the azimuth motor at the specified voltage without feedback. */
  public default void setAzimuthVoltage(double voltage_V) {}

  /** Runs the drive motor at the next pidf voltage based on the given velocity. */
  public default void setNextDriveVelocity(double nextVelocity_radPs) {}

  /** Runs the drive motor at the next pidf voltage based on the given velocity and acceleration. */
  public default void setNextDriveState(
      double nextVelocity_radPs, double nextAcceleration_radPs2) {}

  /** Runs the azimuth motor at the next pidf voltage based on the given position and velocity. */
  public default void setNextAzimuthState(double position_rad, double velocity_radPs) {}
}
