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

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.FieldType;
import frc.robot.util.logging.TunableDouble;
import java.util.function.DoubleSupplier;

public class VisionConstants {
  public static final String logName = "5_Vision";

  // AprilTag layout
  public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(
      RobotConstants.FIELD_TYPE.equals(FieldType.WELDED)
          ? AprilTagFields.k2026RebuiltWelded
          : AprilTagFields.k2026RebuiltAndymark);

  // Camera names, must match names configured on coprocessor
  public static final String[] cameraNames = new String[] {"PC_Camera0", "PC_Camera2"};

  // Camera names for the NT log
  public static final String[] cameraLogNames = cameraNames;

  // Robot to camera transforms
  public static final Transform3d[] robotToCameras = new Transform3d[] {
    new Transform3d(-0.273, 0.311, 0.4, new Rotation3d(0.0, 0, -.30429726 + Math.PI)),
    new Transform3d(-0.273, -0.311, 0.4, new Rotation3d(0.0, 0, .30429726 + Math.PI)) // .30429726
  };

  // Basic filtering thresholds
  public static final double maxAmbiguity = 0.3;
  public static final double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  // public static final double linearStdDevBaseline = 0.02; // Meters
  // public static final double angularStdDevBaseline = 0.06; // Radians
  public static final DoubleSupplier linearStdDevBaseline =
      new TunableDouble("Tuning/" + logName + "/linearStdDevBaseline", .6, () -> true); // Meters
  public static final DoubleSupplier angularStdDevBaseline =
      new TunableDouble("Tuning/" + logName + "/angularStdDevBaseline", 3, () -> true); // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] cameraStdDevFactors = new double[] {
    1.0, // Camera 0
    1.0 // Camera 1
  };
}
