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

import com.thethriftybot.devices.ThriftyNova.Error;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ModuleIO {
  public static class ModuleIOInputs implements LoggableInputs {
    public final ModuleIOInputs.Drive drive = new ModuleIOInputs.Drive();
    public final ModuleIOInputs.AbsoluteEncoder absoluteEncoder =
        new ModuleIOInputs.AbsoluteEncoder();
    public final ModuleIOInputs.Azimuth azimuth = new ModuleIOInputs.Azimuth();
    public final ModuleIOInputs.Odometry odometry = new ModuleIOInputs.Odometry();

    public class Drive {
      public boolean connected = false;
      public final MutAngle position = Radians.mutable(0);
      public final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);
      public final MutVoltage inputVoltage = Volts.mutable(0);
      public final MutVoltage outputVoltage = Volts.mutable(0);
      public final MutCurrent inputCurrent = Amps.mutable(0);
      public final MutCurrent outputCurrent = Amps.mutable(0);
      public final MutVoltage commandedVoltage = Volts.mutable(0);
      public Error[] errors = new Error[0];
    }

    public class AbsoluteEncoder {
      public boolean connected = false;
      public Rotation2d heading = new Rotation2d();
    }

    public class Azimuth {
      public boolean connected = false;
      public final MutAngle position = Radians.mutable(0);
      public final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);
      public final MutVoltage inputVoltage = Volts.mutable(0);
      public final MutVoltage outputVoltage = Volts.mutable(0);
      public final MutCurrent inputCurrent = Amps.mutable(0);
      public final MutCurrent outputCurrent = Amps.mutable(0);
      public final MutVoltage commandedVoltage = Volts.mutable(0);
      public Error[] errors = new Error[0];
    }

    public class Odometry {
      public double[] timestamps = new double[] {};
      public double[] drivePositions_rad = new double[] {};
      public double[] azimuthPositions_rad = new double[] {};
    }

    @Override
    public void toLog(LogTable table) {
      // Drive
      table.put("Drive/.Connected", drive.connected);
      table.put("Drive/Position", drive.position.in(Radians), Radians.name());
      table.put("Drive/Velocity", drive.velocity.in(RadiansPerSecond), RadiansPerSecond.name());
      table.put("Drive/InputVoltage", drive.inputVoltage.in(Volts), Volts.name());
      table.put("Drive/OutputVoltage", drive.outputVoltage.in(Volts), Volts.name());
      table.put("Drive/InputCurrent", drive.inputCurrent.in(Amps), Amps.name());
      table.put("Drive/OutputCurrent", drive.outputCurrent.in(Amps), Amps.name());
      table.put("Drive/CommandedVoltage", drive.commandedVoltage.in(Volts), Volts.name());
      table.put("Drive/Errors", drive.errors);
      // Absolute Encoder
      table.put("AbsoluteEncoder/.Connected", absoluteEncoder.connected);
      table.put("AbsoluteEncoder/Heading", absoluteEncoder.heading.getRadians(), Radians.name());
      // Azimuth
      table.put("Azimuth/.Connected", azimuth.connected);
      table.put("Azimuth/Position", azimuth.position.in(Radians), Radians.name());
      table.put("Azimuth/Velocity", azimuth.velocity.in(RadiansPerSecond), RadiansPerSecond.name());
      table.put("Azimuth/InputVoltage", azimuth.inputVoltage.in(Volts), Volts.name());
      table.put("Azimuth/OutputVoltage", azimuth.outputVoltage.in(Volts), Volts.name());
      table.put("Azimuth/InputCurrent", azimuth.inputCurrent.in(Amps), Amps.name());
      table.put("Azimuth/OutputCurrent", azimuth.outputCurrent.in(Amps), Amps.name());
      table.put("Azimuth/CommandedVoltage", azimuth.commandedVoltage.in(Volts), Volts.name());
      table.put("Azimuth/Errors", azimuth.errors);
      // Odometry
      table.put("Odometry/Timestamps", odometry.timestamps);
      table.put("Odometry/DrivePositions", odometry.drivePositions_rad);
      table.put("Odometry/AzimuthPositions", odometry.azimuthPositions_rad);
    }

    @Override
    public void fromLog(LogTable table) {
      // Drive
      drive.connected = table.get("Drive/.Connected", drive.connected);
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
      drive.commandedVoltage.mut_replace(
          table.get("Drive/CommandedVoltage", drive.commandedVoltage.in(Volts)), Volts);
      drive.errors = table.get("Drive/Errors", drive.errors);

      // Absolute Encoder
      absoluteEncoder.connected =
          table.get("AbsoluteEncoder/.Connected", absoluteEncoder.connected);
      absoluteEncoder.heading = Rotation2d.fromRadians(
          table.get("AbsoluteEncoder/Heading", absoluteEncoder.heading.getRadians()));

      // Azimuth
      azimuth.connected = table.get("Azimuth/.Connected", azimuth.connected);
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
      azimuth.commandedVoltage.mut_replace(
          table.get("Azimuth/CommandedVoltage", azimuth.commandedVoltage.in(Volts)), Volts);
      azimuth.errors = table.get("Azimuth/Errors", azimuth.errors);
      // Odometry
      odometry.timestamps = table.get("Odometry/Timestamps", odometry.timestamps);
      odometry.drivePositions_rad =
          table.get("Odometry/DrivePositions", odometry.drivePositions_rad);
      odometry.azimuthPositions_rad =
          table.get("Odometry/AzimuthPositions", odometry.azimuthPositions_rad);
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
