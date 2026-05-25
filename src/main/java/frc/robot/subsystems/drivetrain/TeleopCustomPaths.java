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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopCustomPaths {
  public PathPlannerPath enterNZ;
  public PathPlannerPath enterAZ;
  public PathPlannerPath test;
  public Command toAZ;
  public Command toNZ;

  private final PathConstraints constraints =
      new PathConstraints(3, 2.5, 3 * Math.PI, 4.95 * Math.PI);

  private double CalculateYMirror(double y, int LRFlip) {
    return (LRFlip == 1) ? 8.07 - y : y;
  }

  private double CalculateXMirror(double x, boolean x_flip) {
    return (x_flip) ? 16.54 - x : x;
  }

  private double CalculateThetaMirror(boolean x_flip) {
    return (x_flip) ? 180 : 0;
  }

  public TeleopCustomPaths(DrivetrainSubsystem drivetrain) {
    double y = drivetrain.getPose().getY();
    boolean x_flip = false;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      x_flip = true;
    }

    int LRFlip;
    if (y <= 4.035) {
      LRFlip = 1;
    } else {
      LRFlip = -1;
    }

    this.toAZ = AutoBuilder.pathfindToPose(
      new Pose2d(
        CalculateXMirror(3.0, x_flip),
        CalculateYMirror(7.408, LRFlip),
        Rotation2d.fromDegrees(CalculateThetaMirror(x_flip))),
        constraints,
        3.0);

    this.toNZ = AutoBuilder.pathfindToPose(
      new Pose2d(
        CalculateXMirror(6.0, x_flip),
        CalculateYMirror(7.408, LRFlip),
        Rotation2d.fromDegrees(CalculateThetaMirror(x_flip))),
        constraints,
        3.0);
  }
}
