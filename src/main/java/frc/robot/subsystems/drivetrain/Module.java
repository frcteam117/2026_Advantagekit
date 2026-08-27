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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Drive;
import frc.robot.subsystems.drivetrain.ModuleIO.ModuleIOInputs;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final Measure<? extends PerUnit<DistanceUnit, AngleUnit>>
      drivePosConversionFactor =
          ((PerUnit<DistanceUnit, AngleUnit>) Meters.per(Radian)).of(Drive.radius_m);
  private static final Measure<? extends PerUnit<LinearVelocityUnit, AngularVelocityUnit>>
      driveVelConversionFactor = ((PerUnit<LinearVelocityUnit, AngularVelocityUnit>)
              MetersPerSecond.per(RadiansPerSecond))
          .of(Drive.radius_m);

  private final ModuleIO io;
  private final ModuleIOInputs inputs = new ModuleIOInputs();
  private final int index;

  // private final Alert driveDisconnectedAlert;
  // private final Alert azimuthDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
  private double lastAzimuthAngle_rad = 0.0;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    // driveDisconnectedAlert = new Alert(
    //     "Disconnected drive motor on module " + Integer.toString(index) + ".", AlertType.kError);
    // azimuthDisconnectedAlert = new Alert(
    //     "Disconnected azimuth motor on module " + Integer.toString(index) + ".",
    // AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(DrivetrainConstants.NAME + "/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometry.timestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometry.drivePositions_rad[i] * Drive.radius_m;
      double angle_rad = inputs.odometry.azimuthPositions_rad[i];
      odometryPositions[i] =
          new SwerveModulePosition(positionMeters, Rotation2d.fromRadians(angle_rad));
    }

    // Update alerts
    // driveDisconnectedAlert.set(!inputs.driveConnected);
    // azimuthDisconnectedAlert.set(!inputs.azimuthConnected);
  }

  // /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  // public void runSetpoint(SwerveModuleState state) {
  //   // Optimize velocity setpoint
  //   state.optimize(Rotation2d.fromRadians(getAngle()));
  //   state.cosineScale(Rotation2d.fromRadians(inputs.azimuthPosition_rad));

  //   // Apply setpoints
  //   io.setNextDriveVelocity(state.speedMetersPerSecond / wheelRadius_m);
  //   io.setNextAzimuthPosition(state.angle.getRadians());
  // }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void setNextState(SwerveModuleState state, double acceleration_mPs2) {
    // Optimize velocity setpoint
    double acceleration_radPs2 =
        Math.cos(state.angle.getRadians() - inputs.absoluteEncoder.heading.getRadians())
            * acceleration_mPs2
            / Drive.radius_m;
    state.optimize(Rotation2d.fromRadians(inputs.absoluteEncoder.heading.getRadians()));
    state.cosineScale(Rotation2d.fromRadians(inputs.absoluteEncoder.heading.getRadians()));

    // Apply setpoints
    io.setNextDriveState(state.speedMetersPerSecond / Drive.radius_m, acceleration_radPs2);
    io.setNextAzimuthState(
        state.angle.getRadians(),
        (state.angle.getRadians() - lastAzimuthAngle_rad) / RobotConstants.CODE_PERIOD_s);
    Logger.recordOutput(
        NAME + "/2_Next/AzimuthVelocity/" + index,
        (state.angle.getRadians() - lastAzimuthAngle_rad) / RobotConstants.CODE_PERIOD_s);
    lastAzimuthAngle_rad = state.angle.getRadians();
  }

  /** Runs the module with the specified voltage while controlling the heading with pure feedback. */
  public void runDriveVoltage(double voltage_V, double heading_rad) {
    io.setDriveVoltage(voltage_V);
    io.setNextAzimuthState(heading_rad, 0.0);
  }

  public void runAzimuthVoltage(double voltage_V) {
    io.setDriveVoltage(0.0);
    io.setAzimuthVoltage(voltage_V);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveVoltage(0.0);
    io.setAzimuthVoltage(0.0);
  }

  /** Returns the current azimuth angle of the module in radians. */
  public Rotation2d getHeading() {
    return inputs.absoluteEncoder.heading;
  }

  /** Returns the current drive position of the module in meters. */
  public Distance getDrivePosition() {
    return inputs.drive.position.timesConversionFactor(drivePosConversionFactor);
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public LinearVelocity getDriveVelocity() {
    return inputs.drive.velocity.timesConversionFactor(driveVelConversionFactor);
  }

  /** Returns the module position (azimuth angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getHeading());
  }

  /** Returns the module state (azimuth angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getHeading());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometry.timestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drive.position.in(Radians);
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.drive.velocity.in(RadiansPerSecond);
  }

  public ModuleIOInputs getInputs() {
    return inputs;
  }

  public void setCoastMode(boolean coast) {
    io.setCoastMode(coast);
  }
}
