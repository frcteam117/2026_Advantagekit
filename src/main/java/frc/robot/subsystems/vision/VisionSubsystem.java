package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import java.util.Arrays;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem {
  //
  private Rotation2d zeroRotation = Rotation2d.kZero;
  // public final PhotonCamera camera0; // needs callibrated
  // public final PhotonCamera camera2;
  //
  AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static List<Pose3d> AprilTagPoses;
  // private final SubsystemCommands subsystemCommands = new SubsystemCommands();
  //
  List<List<PhotonPipelineResult>> curCameraResults;
  //
  public record cameraData<T>(
      Integer AprilTagID,
      Integer cameraNumber,
      Double targetYaw,
      Boolean targetVisible,
      double targetRange,
      double kPVision_Turn) {}
  //
  public int curAprilTagID = 0;
  public double targetYaw = 0.0;
  public double kPVision_Turn = 0.0;
  public double targetRange = 0.0;
  public boolean targetVisible = false;

  public VisionSubsystem(
      DrivetrainSubsystem drivetrain, PhotonCamera camera0, PhotonCamera camera2) {
    AprilTagPoses = Arrays.asList();
    kPVision_Turn = -.03;
    targetYaw = (0.0);
    RobotContainer.getDrivetrain()
        .resetOdometry(RobotContainer.getSubsystemCommands()
            .GetStartPoseFromVisibleAprilTags(drivetrain, camera0, camera2));
    // camera0 = new PhotonCamera("PC_Camera0");
    // camera2 = new PhotonCamera("PC_Camera2");
    // Rotation2d originRot = new Rotation2d(0);
    // Pose2d origin = new Pose2d(0,0,originRot);
    // m_swerve.resetOdometry(origin);
    //

    for (int i = 1; i < 33; i++) { // 33 because 32 tags, index 0 will return a safe Null
      Pose3d tagPose = kTagLayout.getTagPose(i).orElse(new Pose3d());
      SmartDashboard.putNumber("tagPose X", tagPose.getX());
      // AprilTagPoses.add(tagPose);
    }
  }

  //
  public cameraData getCameraResults(PhotonCamera camera2) { // camera0, PhotonCamera camera2) {
    SmartDashboard.putNumber("visionCheck", 000);
    var results = Arrays.asList(
        camera2
            .getAllUnreadResults()); // camera0.getAllUnreadResults(),camera2.getAllUnreadResults());

    int cameraNumber = -1;
    //
    for (int i = 0;
        i < results.size();
        i++) { // looping through results of each camera, with this system camera2 has priority, see
      // if you need to coordinate
      // - it so all cameras combine results or if this system works - THIS IS THE PROBLEM THIS
      // NEVER RETURNS TARGET AND VISIBLE <---------
      cameraNumber = 2; // i;
      if (!results.get(i).isEmpty()) { // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(i).get(results.get(i).size() - 1);
        // SmartDashboard.putNumber("Target tag ID",
        // (result.getTargets().get(result.getTargets().size)-1));
        SmartDashboard.putBoolean("result.hasTargets()", result.hasTargets());
        if (result.hasTargets()) {
          // At least one AprilTag was seen by the camera - should be getting thru to here on/off
          // but still yes
          for (var target : result.getTargets()) {
            // if (aprilTagIDs.contains(target.getFiducialId())) {
            // found one of the tags in aprilTagIDs
            curAprilTagID = target.getFiducialId();
            targetYaw = target.getYaw();
            targetVisible = true;
            SmartDashboard.putNumber("Target tag ID", curAprilTagID);
            SmartDashboard.putNumber("tag vis on camera #", i);
            // System.out.println(target.getYaw());
            targetRange = PhotonUtils.calculateDistanceToTargetMeters( // THESE NEED TO BE TUNED???
                0.5, // Measured with a tape measure, or in CAD.
                1.435, // From 2024 game manual for ID 22, CHANGE IF U WANT TS TO WORK
                Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                Units.degreesToRadians(target.getPitch()));

            // }
          }
        }
      } else {
      }
    }
    //
    return new cameraData(
        curAprilTagID, cameraNumber, targetYaw, targetVisible, targetRange, kPVision_Turn);
  }
}
