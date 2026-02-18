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

package frc.robot.commands;

import com.studica.frc.Navx;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.States.AngularV_State;
import java.util.Arrays;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class SubsystemCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  private static final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
  private static final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
  private static final SlewRateLimiter m_rotLimiter =
      new SlewRateLimiter(9); // import these from robot for better continuity?
  //
  // public Pose2d robotPose2d = new Pose2d();
  // Pose2d robotPose2d = subsystemCommands.GetStartPoseFromVisibleAprilTags(null);
  // ================
  // private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private Rotation2d zeroRotation = Rotation2d.kZero;
  // public final PhotonCamera camera0; // needs callibrated
  // public final PhotonCamera camera2;
  //
  // ===
  // TODO: CHANGE THIS!!!!!!!!!!!!!!
  String allianceColor = "blue";
  static Optional<Alliance> alliance;
  static Boolean alliancePresent = false;

  public SubsystemCommands() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      alliancePresent = true;
    } else {
    }
    /// camera0 = new PhotonCamera("PC_Camera0");
    // camera2 = new PhotonCamera("PC_Camera2");
  }
  // if (alliance.get() ==  Alliance.[Red/Blue]) {
  // };
  //
  // =====

  // ===================================

  public static Command BlankCommand() {
    return Commands.runOnce(() -> {});
  }

  public static Command StopSwerve(
      DrivetrainSubsystem drivetrain, Boolean fieldRelative, Double m_period) {
    // Drivetrain m_swerve,
    return Commands.runOnce(() -> {
      DriveCommands.joystickDrive(
          drivetrain, () -> 0.0, () -> 0.0, () -> 0.0); // add way to stop the robot?????
    });
  }
  //
  public Command resetNavxYaw(Navx navx) {
    return Commands.runOnce(() -> {
      navx.resetYaw();
      System.out.println("reset Navx yaw");
    });
  }
  /*public Command setDrivetrainX(DrivetrainSubsystem drivetrain) {
      return Commands.runOnce( () -> {
          drivetrain.setX();
          System.out.println("set drivetrain X");
      });
  }*/
  // ^^^ from previous code branch, not sure setX is useful here but i'm keeping it till im sure
  public Command AlignToTag(
      DrivetrainSubsystem drivetrain,
      VisionSubsystem vision,
      PhotonCamera camera0,
      PhotonCamera camera1,
      PathCommands pathCommands) {
    return Commands.runOnce(() -> {
      pathCommands.AlignToTag(drivetrain, vision, camera0, camera1);
    });
  }

  // =================================== // unclear is these will center here or stem from their
  // individual subsystems?
  // for now run this constantly, add different robot states later ^^
  public Command RunAdjustShooterForDistanceFromHub(
      ShooterSubsystem shooter, Double distanceFromHub) {
    return Commands.run(() -> {
      double dutyCycle = ShooterConstants.lerpTable.get(distanceFromHub);
      shooter.setFlywheelGoal(
          new AngularV_State(200)); // does this adjust for differences between the two wheels?
      // TODO: ask max how this works
      // shooter.setRightShooterDutyCycle(dutyCycle);
    });
  }

  /*// non-drivetrain subsystem commands:
  public static Command ExpandHopper() {
      return Commands.runOnce( () -> {
          hopperSubsystem.setAngle(HopperConstants.EXPANDED_ANGLE);
      });
  }
  public static Command RetractHopper() {
      return Commands.runOnce( () -> {
          hopperSubsystem.setAngle(HopperConstants.RETRACTED_ANGLE);
      });
  }
  //===
  public static Command SetHoodAngleManual(Angle angle) {
      return Commands.runOnce( () -> {
          hoodSubsystem.setAngle(angle);
      });
  }// FIX VVVVVVVVVVVVVVVVV
  public static Command SetHoodAngleAuto(Double targetPitch) {
      double target = Units.radiansToDegrees(targetPitch); // if its in radians??? i dont actually know
      if (target > HoodConstants.MAX_ANGLE_DEGREES) {
          target = HoodConstants.MAX_ANGLE_DEGREES;
      }
      if (target < HoodConstants.MIN_ANGLE_DEGREES) {
          target = HoodConstants.MAX_ANGLE_DEGREES;
      }
      return Commands.runOnce( () -> {
          if (alliancePresent) {
              if (alliance.get() == Alliance.Red) {
                  // HoodSubsystem.setAngle(Degrees.of(target));
              }
              else if (alliance.get() == Alliance.Blue) {
                  //HoodSubsystem.setAngle(Degrees.of(target));
              }
          }
          // put auto figure out how far from hopper code here???
      });
  }
  //===
  public static Command DeployIntake() {
      return Commands.runOnce( () -> {
          intakeSubsystem.setIntakeDeployAngle(ClimberConstants.FULLY_DEPLOYED_ANGLE);
      });
  }
  public static Command UndeployIntake() {// should this be RetractIntake instead?
      return Commands.runOnce( () -> {
          intakeSubsystem.setIntakeDeployAngle(ClimberConstants.FULLY_UNDEPLOYED_ANGLE);
      });
  }
  public static Command IntakeFuel() { //
      return Commands.runOnce( () -> {
          intakeSubsystem.setIntakeDutyCycle(1);
      });
  }
  public static Command OuttakeFuel() { //
      return Commands.runOnce( () -> {
          intakeSubsystem.setIntakeDutyCycle(-1);
      });
  }
  public static Command StopIntake() { //
      return Commands.runOnce( () -> {
          intakeSubsystem.setIntakeDutyCycle(0);
      });
  }
  //===
  public static Command RunKicker() { //
      return Commands.runOnce( () -> {
          //intakeSubsystem.setIntakeDutyCycle(0);
      });
  }
  public static Command StopKicker() { //
      return Commands.runOnce( () -> {
          //intakeSubsystem.setIntakeDutyCycle(0);
      });
  }
  //
  public static Command RunLeftShooter() { // adjust this?
      return Commands.runOnce( () -> {
          shooterSubsystem.setLeftShooterDutyCycle(1);
      });
  }
  public static Command RunRightShooter() { // adjust this?,
      return Commands.runOnce( () -> {
          shooterSubsystem.setRightShooterDutyCycle(1);
      });
  }
  public static Command StopLeftShooter() { // adjust this?
      return Commands.runOnce( () -> {
          shooterSubsystem.setLeftShooterDutyCycle(0);
      });
  }
  public static Command StopRightShooter() { // adjust this?,
      return Commands.runOnce( () -> {
          shooterSubsystem.setRightShooterDutyCycle(0);
      });
  }
  //===
  public static Command TowerAlign(String position) { // position will be like front left/center/right or side or back yknow
      return Commands.runOnce( () -> {});
  }
  //===
  public static Command ExtendClimber() { // dunno about this one
      return Commands.runOnce( () -> {});
  }
  public static Command RetractClimber() { // dunno about this one
      return Commands.runOnce( () -> {});
  }
  public static Command ClimbLevel1() {
      return Commands.runOnce( () -> {});
  }
  //public static Command ClimbLevel3() {
  //===
  */
  // ADJUST FOR CAMERA POSITION. CHECK WHICH CAMERA AND ADD OFFSET FOR CAMERA POSEs!!!!!
  public Pose2d GetStartPoseFromVisibleAprilTags(
      DrivetrainSubsystem drivetrain,
      PhotonCamera camera0,
      PhotonCamera
          camera2) { // List<List<PhotonPipelineResult>> results) { // (only from start for now)
    // change to be for any point in game by making it dependent on the alliance side if
    // - at start and the odometry robot pose at any other point???
    var results = Arrays.asList(camera0.getAllUnreadResults(), camera2.getAllUnreadResults());
    SmartDashboard.putNumber("visionCheck", 111);
    Pose2d robotPose = null;
    int curAprilTagID;
    double targetYaw;
    double targetRange;
    double robotX = 0;
    double robotY = 0;
    Rotation2d robotHeading = new Rotation2d();
    for (int i = 0;
        i < results.size();
        i++) { // looping through results of each camera, with this system camera2 has priority, see
      // if you need to coordinate
      // - it so all cameras combine results or if this system works - THIS IS THE PROBLEM THIS
      // NEVER RETURNS TARGET AND VISIBLE <---------
      if (!results.get(i).isEmpty()) { // Camera processed a new frame since last
        // Get the last one in the list.
        SmartDashboard.putNumber("visionCheck", 222);
        var result = results.get(i).get(results.get(i).size() - 1);
        SmartDashboard.putNumber("visionCheck", 222.5);
        // SmartDashboard.p utNumber("Target tag ID",
        // (result.getTargets().get(result.getTargets().size)-1));
        SmartDashboard.putBoolean("result.hasTargets()", result.hasTargets());
        SmartDashboard.putString("camera result", result.toString());
        if (result.hasTargets()) { // PROBLEM HERE PROBLEM HERE PROBLEM HERE
          SmartDashboard.putNumber("visionCheck", 333);

          // At least one AprilTag was seen by the camera - should be getting thru to here on/off
          // but still yes
          for (var target : result.getTargets()) {
            SmartDashboard.putString("camera result target(s)", target.toString());
            SmartDashboard.putNumber("visionCheck", 444);
            // if (aprilTagIDs.contains(target.getFiducialId())) {
            // found one of the tags in aprilTagIDs
            curAprilTagID = target.getFiducialId();
            targetYaw = target.getYaw();
            SmartDashboard.putNumber("Target tag ID", curAprilTagID);
            SmartDashboard.putNumber("tag vis on camera #", i);
            System.out.println(target.getYaw());
            targetRange = PhotonUtils.calculateDistanceToTargetMeters( // THESE NEED TO BE TUNED???
                0.5, // Measured with a tape measure, or in CAD.
                RobotContainer.AprilTagPoses.get(curAprilTagID).getZ(),
                Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                Units.degreesToRadians(target.getPitch()));
            //

            if (alliancePresent) {
              SmartDashboard.putBoolean("alliance present", true);
              if (alliance.get() == Alliance.Red) { // if on red side, add x,y,rot
                robotX = (drivetrain.getPose().getX() + (targetRange * Math.sin(targetYaw)));
                robotY = (drivetrain.getPose().getX() + (targetRange * Math.cos(targetYaw)));
                robotHeading = Rotation2d.fromDegrees(0 - targetYaw);
              } else if (alliance.get() == Alliance.Blue) { // if on blue side, subtract x,y,rot
                robotX = (drivetrain.getPose().getX() - (targetRange * Math.sin(targetYaw)));
                robotY = (drivetrain.getPose().getX() - (targetRange * Math.cos(targetYaw)));
                robotHeading = Rotation2d.fromDegrees(180 - targetYaw);
              }
            } else {
              SmartDashboard.putBoolean("alliance present", false);
            }

            robotPose = new Pose2d(robotX, robotY, robotHeading);
          }
        } else {
          SmartDashboard.putNumber("visionCheck", 555); // this is what is showing when it quits
        }
        // }
      } else {
      }
    }

    //
    if (robotPose != null) {
      SmartDashboard.putNumber("robot pose from tag X", robotX);
      SmartDashboard.putNumber("robot pose from tag Y", robotY);
      return robotPose;
    } else {
      return new Pose2d();
    }
  }

  public Command LogStartPoseFromVisibleAprilTags(
      DrivetrainSubsystem drivetrain,
      PhotonCamera camera0,
      PhotonCamera
          camera2) { // List<List<PhotonPipelineResult>> results) { // (only from start for now)
    // change to be for any point in game by making it dependent on the alliance side if
    // - at start and the odometry robot pose at any other point???
    // System.out.println(17171717);
    return Commands.runOnce(() -> {
      var results = Arrays.asList(
          camera2
              .getAllUnreadResults()); // (camera0.getAllUnreadResults(),camera2.getAllUnreadResults());
      // change to not use camera0 while its not on robot?
      Pose2d robotPose = null;
      int curAprilTagID;
      double targetYaw;
      double targetRange;
      double robotX = 0;
      double robotY = 0;
      Rotation2d robotHeading = new Rotation2d();
      for (int i = 0; i < results.size(); i++) {
        SmartDashboard.putNumber("visionCheck", 0.555);

        // looping through results of each camera, with this system camera2 has priority, see if you
        // need to coordinate
        // - it so all cameras combine results or if this system works - THIS IS THE PROBLEM THIS
        // NEVER RETURNS TARGET AND VISIBLE <---------
        if (!results.get(i).isEmpty()) { // Camera processed a new frame since last
          // Get the last one in the list.
          SmartDashboard.putNumber("visionCheck", 222);
          var result = results.get(i).get(results.get(i).size() - 1); // get latest result
          SmartDashboard.putNumber("visionCheck", 222.555);
          // SmartDashboard.putNumber("Target tag ID",
          // (result.getTargets().get(result.getTargets().size)-1));
          SmartDashboard.putBoolean("result.hasTargets()", result.hasTargets());
          SmartDashboard.putString("camera result", result.toString());

          // TO DEBUG: camera IS getting results for the tag, but this isnt working for some
          // reason :(
          if (result.hasTargets()) { // PROBLEM HERE PROBLEM HERE PROBLEM HERE
            SmartDashboard.putNumber("visionCheck", 333);

            // At least one AprilTag was seen by the camera - should be getting thru to here on/off
            // but still yes
            for (var target : result.getTargets()) {
              SmartDashboard.putString("camera result target(s)", target.toString());
              SmartDashboard.putNumber("visionCheck", 444);
              // if (aprilTagIDs.contains(target.getFiducialId())) {
              // found one of the tags in aprilTagIDs
              curAprilTagID = target.getFiducialId();
              targetYaw = target.getYaw();
              SmartDashboard.putNumber("Target tag ID", curAprilTagID);
              SmartDashboard.putNumber("tag vis on camera #", i);
              System.out.println(target.getYaw());
              targetRange =
                  PhotonUtils.calculateDistanceToTargetMeters( // THESE NEED TO BE TUNED???
                      0.5, // Measured with a tape measure, or in CAD.
                      RobotContainer.AprilTagPoses.get(curAprilTagID).getZ(),
                      Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                      Units.degreesToRadians(target.getPitch()));
              //

              if (true) {
                SmartDashboard.putBoolean("alliance present", true);
                if (allianceColor == "red") { // if on red side, add x,y,rot
                  robotX = (drivetrain.getPose().getX() + (targetRange * Math.sin(targetYaw)));
                  robotY = (drivetrain.getPose().getX() + (targetRange * Math.cos(targetYaw)));
                  robotHeading = Rotation2d.fromDegrees(0 - targetYaw);
                } else if (allianceColor == "blue") { // if on blue side, subtract x,y,rot
                  robotX = (drivetrain.getPose().getX() - (targetRange * Math.sin(targetYaw)));
                  robotY = (drivetrain.getPose().getX() - (targetRange * Math.cos(targetYaw)));
                  robotHeading = Rotation2d.fromDegrees(180 - targetYaw);
                }
              } else {
                SmartDashboard.putBoolean("alliance present", false);
              }

              robotPose = new Pose2d(robotX, robotY, robotHeading);
            }
          } else {
          }
          // }
        } else {
        }
      }

      //
      if (robotPose != null) {
        SmartDashboard.putNumber("robot pose from tag X", robotX);
        SmartDashboard.putNumber("robot pose from tag Y", robotY);
        // return robotPose;
      } else {
        // return null;
      }
    });
  }
  //

  //
  public Command ExampleSequence() {
    return Commands.sequence(Commands.run(() -> {}));
  }
}
