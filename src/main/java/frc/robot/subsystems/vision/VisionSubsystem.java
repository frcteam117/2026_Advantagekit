package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.angularStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.angularStdDevMegatag2Factor;
import static frc.robot.subsystems.vision.VisionConstants.cameraStdDevFactors;
import static frc.robot.subsystems.vision.VisionConstants.linearStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.linearStdDevMegatag2Factor;
import static frc.robot.subsystems.vision.VisionConstants.maxAmbiguity;
import static frc.robot.subsystems.vision.VisionConstants.maxZError;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem {
  //
  private Rotation2d zeroRotation = Rotation2d.kZero;
  // public final PhotonCamera camera0; // needs callibrated
  // public final PhotonCamera camera2;
  //
  AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
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
  public int updateNum = 1;
  //
  DrivetrainSubsystem m_drivetrain;
  public PhotonCamera m_camera0;
  public PhotonCamera m_camera2;
  public PhotonPoseEstimator m_estimatorCam0;
  public PhotonPoseEstimator m_estimatorCam1;

  //
  public VisionSubsystem(VisionConsumer consumer, VisionIO... io) {
    m_drivetrain = RobotContainer.getDrivetrain();
    this.m_camera0 = RobotContainer.camera0; // io.get(0).camera;
    this.m_camera2 = RobotContainer.camera1; // io.get(1).camera;
    this.m_estimatorCam0 = RobotContainer.photonEstimatorCam0;
    this.m_estimatorCam1 = RobotContainer.photonEstimatorCam1;
    //
    AprilTagPoses = Arrays.asList();
    kPVision_Turn = -.03;
    targetYaw = (0.0);
    for (int i = 1; i < 33; i++) { // 33 because 32 tags, index 0 will return a safe Null
      Pose3d tagPose = aprilTagLayout.getTagPose(i).orElse(new Pose3d());
      SmartDashboard.putNumber("tagPose X", tagPose.getX());
      // AprilTagPoses.add(tagPose);
    }
    //
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] = new Alert(
          "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /*public VisionSubsystem(
      DrivetrainSubsystem drivetrain,
      PhotonCamera camera0,
      PhotonCamera camera2,
      PhotonPoseEstimator estimatorCam0,
      PhotonPoseEstimator estimatorCam1) {

    AprilTagPoses = Arrays.asList();
    kPVision_Turn = -.03;
    targetYaw = (0.0);
    m_drivetrain = drivetrain;
    this.m_camera0 = camera0;
    this.m_camera2 = camera2;
    this.m_estimatorCam0 = estimatorCam0;
    this.m_estimatorCam1 = estimatorCam1;
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
  }*/

  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public void VisionPeriodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("5_Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
            || (observation.tagCount() == 1
                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
            || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

            // Must be within the field boundaries
            || observation.pose().getX() < 0.0
            || observation.pose().getX() > aprilTagLayout.getFieldLength()
            || observation.pose().getY() < 0.0
            || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput(
          "5_Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "5_Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "5_Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "5_Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "5_Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "5_Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "5_Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "5_Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
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

  //

  /*public void VisionPeriodic() { // updateNum > 0
    if (updateNum > 2) {
      updateNum = 1;
    }
    var visionEst = RobotContainer.getSubsystemCommands()
        .GetEstimatedRobotPoseFromVisibleAprilTags(
            m_drivetrain, m_camera0, m_camera2, m_estimatorCam0, m_estimatorCam1);
    if (!visionEst.isEmpty()) {
      var optionalEst = visionEst.get(updateNum - 1); // add fallback for index errors?
      if (!optionalEst.isEmpty()) {
        EstimatedRobotPose est =
            visionEst.get(updateNum - 1).orElse(new EstimatedRobotPose(null, 0.0, null));
        Pose3d visionEstPose3d = est.estimatedPose;
        Pose2d visionEstPose2d = new Pose2d(
            visionEstPose3d.getX(),
            visionEstPose3d.getY(),
            visionEstPose3d.getRotation().toRotation2d());
        //
        RobotContainer.getDrivetrain().accept(visionEstPose2d);
        updateNum += 1;
      }
    }
  }*/
  // ===
}
