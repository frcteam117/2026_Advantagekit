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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.Pivot;
import frc.robot.subsystems.intake.PivotConstants;
import frc.robot.subsystems.intake.Roller;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.states.premade.RadVel_State;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

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
  public static Command AlignToTag(
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
      shooter.setFlywheelGoal( // use 600 as radians baseline, ask if this is ok
          new RadVel_State(dutyCycle)); // does this adjust for differences between the two wheels?
      // TODO: ask max how this works
      // shooter.setRightShooterDutyCycle(dutyCycle);
    });
  } // figure out the command start/finish parameter thingies ig

  public Command FireStartFuel(ShooterSubsystem shooter) {
    return Commands.run(() -> {
          shooter.setFlywheelGoal( // use 600 as radians baseline, ask if this is ok
              new RadVel_State(
                  600 * 0.7)); // does this adjust for differences between the two wheels?
          // TODO: ask max how this works, also make a constants file for these values?
          // shooter.setRightShooterDutyCycle(dutyCycle);
        })
        .withTimeout(5); // is time the best way to manage this? idk: ASK
  }

  /*// non-drivetrain subsystem commands:
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
  */

  public static Command DeployIntake(Pivot pivot) {
    return Commands.runOnce(() -> {
      pivot.setAngleWithRioProfile(PivotConstants.MechanismConstants.kMaxSoftLimit);
    });
  }

  public static Command UndeployIntake(Pivot pivot) { // should this be RetractIntake instead?
    return Commands.runOnce(() -> {
      pivot.setAngleWithRioProfile(
          PivotConstants.MechanismConstants
              .kMinSoftLimit); // should this be the hardlimit or something else?
    });
  }

  public static Command IntakeFuel(Roller roller) { //
    return Commands.runOnce(() -> {
      roller.set(1); // set to adjust for distance and everything
    });
  }

  public static Command OuttakeFuel(Roller roller) { //
    return Commands.runOnce(() -> {
      roller.set(-1);
    });
  }

  public static Command StopIntake(Roller roller) { //
    return Commands.runOnce(() -> {
      roller.set(0); // run PID on all of this so it doesn't break? or is that already happening?
    });
  }
  /*
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
  public List<Optional<EstimatedRobotPose>> GetEstimatedRobotPoseFromVisibleAprilTags(
      VisionSubsystem
          vision) { // List<List<PhotonPipelineResult>> results) { // (only from start for
    // now)
    // change to be for any point in game by making it dependent on the alliance side if
    // - at start and the odometry robot pose at any other point???
    PhotonCamera camera0 = vision.m_camera0;
    PhotonCamera camera2 = vision.m_camera2;
    PhotonPoseEstimator estimatorCam0 = vision.m_estimatorCam0;
    PhotonPoseEstimator estimatorCam1 = vision.m_estimatorCam1;
    var results = Arrays.asList(camera0.getAllUnreadResults(), camera2.getAllUnreadResults());
    List<Optional<EstimatedRobotPose>> visionEstimates = Arrays.asList();

    for (int i = 0; i < results.size(); i++) {
      if (!results.get(i).isEmpty()) { // Camera processed a new frame since last
        // Get the last one in the list.
        PhotonPoseEstimator estimator;
        if (i == 0) {
          estimator = estimatorCam0;
        } else {
          estimator = estimatorCam1;
        }
        var result = results.get(i).get(results.get(i).size() - 1);
        Optional<EstimatedRobotPose> visionEst = estimator.estimateCoprocMultiTagPose(result);
        if (visionEst.isEmpty()) {
          visionEst = estimator.estimateLowestAmbiguityPose(result);
        }
        visionEstimates.add(visionEst);
      }
    }
    //

    // SmartDashboard.putNumber("robot pose from tag X", robotX);
    // SmartDashboard.putNumber("robot pose from tag Y", robotY);
    return visionEstimates;
  }

  public Command ExampleSequence() {
    return Commands.sequence(Commands.run(() -> {}));
  }
}
