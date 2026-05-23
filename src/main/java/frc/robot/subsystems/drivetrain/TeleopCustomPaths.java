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

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class TeleopCustomPaths {
  public PathPlannerPath enterNZ;
  public PathPlannerPath enterAZ;

  private double CalculateMirror(double value, int LRFlip) {
    if (LRFlip == 1) {
      return value;
    } else {
      return 8.07 - value;
    }
  }

  public TeleopCustomPaths(DrivetrainSubsystem drivetrain) {
    double x = drivetrain.getPose().getX();
    double y = drivetrain.getPose().getY();
    Rotation2d rotation = drivetrain.getPose().getRotation();
    int LRFlip = 1;
    if (y <= 4.035) {
      LRFlip = -1;
    }
    List<Waypoint> waypointsEnterNZ = PathPlannerPath.waypointsFromPoses(
        new Pose2d(x, y, rotation),
        new Pose2d(3.343, CalculateMirror(7.457, LRFlip), Rotation2d.fromDegrees(0)),
        new Pose2d(4.539, CalculateMirror(7.457, LRFlip), Rotation2d.fromDegrees(0)),
        new Pose2d(5.652, CalculateMirror(7.457, LRFlip), Rotation2d.fromDegrees(0)));
    List<Waypoint> waypointsEnterAZ = PathPlannerPath.waypointsFromPoses(
        new Pose2d(x, y, rotation),
        new Pose2d(6.092, CalculateMirror(7.508, LRFlip), Rotation2d.fromDegrees(0)),
        new Pose2d(4.833, CalculateMirror(7.508, LRFlip), Rotation2d.fromDegrees(0)),
        new Pose2d(3.473, CalculateMirror(7.381, LRFlip), Rotation2d.fromDegrees(0)),
        new Pose2d(2.888, CalculateMirror(7.046, LRFlip), Rotation2d.fromDegrees(117 * LRFlip)));
    PathConstraints constraints = new PathConstraints(5.0, 3.5, 3 * Math.PI, 4.95 * Math.PI);
    enterNZ = new PathPlannerPath(
        waypointsEnterNZ, constraints, null, new GoalEndState(3, Rotation2d.fromDegrees(0)));
    enterAZ = new PathPlannerPath(
        waypointsEnterAZ,
        constraints,
        null,
        new GoalEndState(0.0, Rotation2d.fromDegrees(117 * LRFlip)));
  }
}
