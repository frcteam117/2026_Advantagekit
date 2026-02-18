package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // add camera position offsets for pose estimation once teh cameras have been placed??
  // :3
  // and definitions for where certain tags are on the field
  // - (like
  //      int RED_ALLIANCE_ZONE_HUB_FRONT_LEFT_TAG_ID = 9;
  //      int BLUE_NEUTRAL_ZONE_HUB_SIDE_RIGHT_TAG_ID = 21;
  //    etc.
  // - though that's a lot of underscores o-o)
  public static final String camera0Name = "PC_Camera0";
  public static final String camera1Name = "PC_Camera2";

  public static Transform3d robotToCamera0 =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  public VisionConstants() {}
}
