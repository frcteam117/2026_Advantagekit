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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.components.bases.ComponentStates.AbsoluteEncoder_State;
import frc.robot.util.components.bases.ComponentStates.Motor_State;
import frc.robot.util.states.LoggableStateInputs;
import frc.robot.util.states.State;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ModuleIO {
  public static class ModuleIOInputs implements LoggableStateInputs {
    public Motor_State driveMotor_State = new Motor_State(0, 0, 0, 0, 0, 0);
    public AbsoluteEncoder_State azimuthAbsolutePosition_State = new AbsoluteEncoder_State(0);
    public Motor_State azimuthMotor_State = new Motor_State(0, 0, 0, 0, 0, 0);

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

  public static class ModifiedModuleIOInputs implements LoggableInputs {
    public final ModifiedModuleIOInputs.Drive drive = new ModifiedModuleIOInputs.Drive();
    public final ModifiedModuleIOInputs.AbsoluteEncoder absoluteEncoder =
        new ModifiedModuleIOInputs.AbsoluteEncoder();
    public final ModifiedModuleIOInputs.Azimuth azimuth = new ModifiedModuleIOInputs.Azimuth();
    public final ModifiedModuleIOInputs.Odometry odometry = new ModifiedModuleIOInputs.Odometry();

    public class Drive {
      public final MutAngle position = Radians.mutable(0);
      public final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);
      public final MutVoltage inputVoltage = Volts.mutable(0);
      public final MutVoltage outputVoltage = Volts.mutable(0);
      public final MutCurrent inputCurrent = Amps.mutable(0);
      public final MutCurrent outputCurrent = Amps.mutable(0);
    }

    public class AbsoluteEncoder {
      public Rotation2d heading = new Rotation2d();
    }

    public class Azimuth {
      public final MutAngle position = Radians.mutable(0);
      public final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);
      public final MutVoltage inputVoltage = Volts.mutable(0);
      public final MutVoltage outputVoltage = Volts.mutable(0);
      public final MutCurrent inputCurrent = Amps.mutable(0);
      public final MutCurrent outputCurrent = Amps.mutable(0);
    }

    public class Odometry {
      public double[] odometryTimestamps = new double[] {};
      public double[] odometryDrivePositions_rad = new double[] {};
      public double[] odometryAzimuthPositions_rad = new double[] {};
    }

    @Override
    public void toLog(LogTable table) {
      // Drive
      table.put("Drive/Position", drive.position.in(Radians), Radians.name());
      table.put("Drive/Velocity", drive.velocity.in(RadiansPerSecond), RadiansPerSecond.name());
      table.put("Drive/InputVoltage", drive.inputVoltage.in(Volts), Volts.name());
      table.put("Drive/OutputVoltage", drive.outputVoltage.in(Volts), Volts.name());
      table.put("Drive/InputCurrent", drive.inputCurrent.in(Amps), Amps.name());
      table.put("Drive/OutputCurrent", drive.outputCurrent.in(Amps), Amps.name());
      // Absolute Encoder
      table.put("Azimuth/Heading", absoluteEncoder.heading.getRadians(), Radians.name());
      // Azimuth
      table.put("Azimuth/Position", azimuth.position.in(Radians), Radians.name());
      table.put("Azimuth/Velocity", azimuth.velocity.in(RadiansPerSecond), RadiansPerSecond.name());
      table.put("Azimuth/InputVoltage", azimuth.inputVoltage.in(Volts), Volts.name());
      table.put("Azimuth/OutputVoltage", azimuth.outputVoltage.in(Volts), Volts.name());
      table.put("Azimuth/InputCurrent", azimuth.inputCurrent.in(Amps), Amps.name());
      table.put("Azimuth/OutputCurrent", azimuth.outputCurrent.in(Amps), Amps.name());
      // Odometry
      table.put("Odometry/Timestamps", odometry.odometryTimestamps);
      table.put("Odometry/DrivePositions", odometry.odometryDrivePositions_rad);
      table.put("Odometry/AzimuthPositions", odometry.odometryAzimuthPositions_rad);
    }

    @Override
    public void fromLog(LogTable table) {
      // Drive
      drive.position.mut_replace(table.get("Drive/Position", drive.position.in(Radians)), Radians);
      drive.velocity.mut_replace(
          table.get("Drive/Velocity", drive.velocity.in(RadiansPerSecond)), RadiansPerSecond);
      drive.inputVoltage.mut_replace(
          table.get("Drive/InputVoltage", drive.inputVoltage.in(Volts)), Volts);
      drive.outputVoltage.mut_replace(
          table.get("Drive/OutputVoltage", drive.outputVoltage.in(Volts)), Volts);
      drive.inputCurrent.mut_replace(
          table.get("Drive/InputCurrent", drive.inputCurrent.in(Amps)), Amps);
      drive.outputCurrent.mut_replace(
          table.get("Drive/OutputCurrent", drive.outputCurrent.in(Amps)), Amps);
      // Absolute Encoder
      absoluteEncoder.heading = Rotation2d.fromRadians(
          table.get("Azimuth/Heading", absoluteEncoder.heading.getRadians()));
      // Azimuth
      azimuth.position.mut_replace(
          table.get("Azimuth/Position", azimuth.position.in(Radians)), Radians);
      azimuth.velocity.mut_replace(
          table.get("Azimuth/Velocity", azimuth.velocity.in(RadiansPerSecond)), RadiansPerSecond);
      azimuth.inputVoltage.mut_replace(
          table.get("Azimuth/InputVoltage", azimuth.inputVoltage.in(Volts)), Volts);
      azimuth.outputVoltage.mut_replace(
          table.get("Azimuth/OutputVoltage", azimuth.outputVoltage.in(Volts)), Volts);
      azimuth.inputCurrent.mut_replace(
          table.get("Azimuth/InputCurrent", azimuth.inputCurrent.in(Amps)), Amps);
      azimuth.outputCurrent.mut_replace(
          table.get("Azimuth/OutputCurrent", azimuth.outputCurrent.in(Amps)), Amps);
      // Odometry
      odometry.odometryTimestamps = table.get("Odometry/Timestamps", odometry.odometryTimestamps);
      odometry.odometryDrivePositions_rad =
          table.get("Odometry/DrivePositions", odometry.odometryDrivePositions_rad);
      odometry.odometryAzimuthPositions_rad =
          table.get("Odometry/AzimuthPositions", odometry.odometryAzimuthPositions_rad);
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
