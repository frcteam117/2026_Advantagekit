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

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearAcceleration;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GyroIO {
  public static class GyroIOInputs implements LoggableInputs {
    public boolean connected = false;
    // public Rotation2d yaw = new Rotation2d();
    public Rotation3d orientation = new Rotation3d();
    public final MutAngularVelocity[] angularVel = new MutAngularVelocity[] {
      RadiansPerSecond.mutable(0), RadiansPerSecond.mutable(0), RadiansPerSecond.mutable(0)
    };
    public final MutLinearAcceleration[] linearAcc = new MutLinearAcceleration[] {
      MetersPerSecondPerSecond.mutable(0),
      MetersPerSecondPerSecond.mutable(0),
      MetersPerSecondPerSecond.mutable(9.81)
    };
    // public double yawVelocityRadPerSec = 0.0;
    public final MutAngle angleFromHorizontal = Radians.mutable(0);
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};

    @Override
    public void toLog(LogTable table) {
      table.put(".Connected", connected);
      table.put("Orientation/Roll", orientation.getX(), Radians.name());
      table.put("Orientation/Pitch", orientation.getY(), Radians.name());
      table.put("Orientation/Yaw", orientation.getZ(), Radians.name());
      table.put("AngularVel/X_yz", angularVel[0].in(RadiansPerSecond), RadiansPerSecond.name());
      table.put("AngularVel/Y_zx", angularVel[1].in(RadiansPerSecond), RadiansPerSecond.name());
      table.put("AngularVel/Z_xy", angularVel[2].in(RadiansPerSecond), RadiansPerSecond.name());
      table.put(
          "LinearAcc/X",
          linearAcc[0].in(MetersPerSecondPerSecond),
          MetersPerSecondPerSecond.name());
      table.put(
          "LinearAcc/Y",
          linearAcc[1].in(MetersPerSecondPerSecond),
          MetersPerSecondPerSecond.name());
      table.put(
          "LinearAcc/Z",
          linearAcc[2].in(MetersPerSecondPerSecond),
          MetersPerSecondPerSecond.name());
      table.put("AngleFromHorizontal", angleFromHorizontal.in(Radians), Radians.name());
      table.put("OdometryYawTimestamps", odometryYawTimestamps);
      table.put("OdometryYawPositions", odometryYawPositions);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get(".Connected", connected);
      orientation = new Rotation3d(
          table.get("Orientation/Roll", orientation.getX()),
          table.get("Orientation/Pitch", orientation.getY()),
          table.get("Orientation/Yaw", orientation.getZ()));
      angularVel[0].mut_replace(
          table.get("AngularVel/X_yz", angularVel[0].in(RadiansPerSecond)), RadiansPerSecond);
      angularVel[1].mut_replace(
          table.get("AngularVel/Y_zx", angularVel[1].in(RadiansPerSecond)), RadiansPerSecond);
      angularVel[2].mut_replace(
          table.get("AngularVel/Z_xy", angularVel[2].in(RadiansPerSecond)), RadiansPerSecond);
      linearAcc[0].mut_replace(
          table.get("LinearAcc/X", linearAcc[0].in(MetersPerSecondPerSecond)),
          MetersPerSecondPerSecond);
      linearAcc[1].mut_replace(
          table.get("LinearAcc/Y", linearAcc[1].in(MetersPerSecondPerSecond)),
          MetersPerSecondPerSecond);
      linearAcc[2].mut_replace(
          table.get("LinearAcc/Z", linearAcc[2].in(MetersPerSecondPerSecond)),
          MetersPerSecondPerSecond);
      angleFromHorizontal.mut_replace(
          table.get("AngleFromHorizontal", angleFromHorizontal.in(Radians)), Radians);
      odometryYawTimestamps = table.get("OdometryYawTimestamps", odometryYawTimestamps);
      odometryYawPositions = table.get("OdometryYawPositions", odometryYawPositions);
    }
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
