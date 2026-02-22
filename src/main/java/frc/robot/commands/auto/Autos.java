package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathCommands;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
//
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public final class Autos {
  Pose2d robotStartPose;
  static Optional<Alliance> alliance;
  static Boolean alliancePresent = false;
  //
  private Autos(
      DrivetrainSubsystem drivetrain,
      SubsystemCommands subsystemCommands,
      PhotonCamera camera0,
      PhotonCamera camera2,
      PhotonPoseEstimator estimatorCam0,
      PhotonPoseEstimator estimatorCam1) {
    // throw new UnsupportedOperationException("don't use this dummy");
    // robotStartPose =
    // this is handled in vision periodic:
    //    subsystemCommands.GetEstimatedRobotPoseFromVisibleAprilTags(
    //        drivetrain, camera0, camera2, estimatorCam0, estimatorCam1);
    // drivetrain.resetOdometry(robotStartPose);
    //
    alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      alliancePresent = true;
    } else {
    }
  }
  // do we have to pass pathCommands? IDEFKATPBRO
  public Command Auto1(
      DrivetrainSubsystem drivetrain,
      PathCommands pathCommands,
      Boolean fieldRelative,
      Double m_period,
      Robot robot,
      Double targetYaw) { // figure out how running this is gonna work,
    // - you'll probably need to get rid of the parameters and have the Autos.java file
    // - deal with it itself

    // DIFFERENTIATE THE AUTO HERE: ADD POSES FOR WHOLE AUTOS IN HERE vvv

    // ========================================
    List<Pose2d> targetPoses = AutoPoses.AUTO1_POSE2DS;
    return Commands.sequence( // drive to hub (with offset)
        Commands.run(() -> {
              List<Double> values = pathCommands.CalcSwerveValues(
                  drivetrain.getPose(), targetPoses.get(0)); // change 0
              DriveCommands.joystickDrive(
                  drivetrain, () -> values.get(0), () -> values.get(1), () -> values.get(2));
            })
            .until(() -> pathCommands.CloseEnough(drivetrain.getPose(), targetPoses.get(0))),
        //
        Commands.run(() -> {
              // run shooter for 5 seconds (/fire 8 fuel)
            })
            .withTimeout(5),
        Commands.run(
                () -> { // go thru to neutral zone
                  List<Double> values = pathCommands.CalcSwerveValues(
                      drivetrain.getPose(), targetPoses.get(1)); // change 1
                  DriveCommands.joystickDrive(
                      drivetrain, () -> values.get(0), () -> values.get(1), () -> values.get(2));
                })
            .until(() -> pathCommands.CloseEnough(drivetrain.getPose(), targetPoses.get(1))),
        Commands.run(
                () -> { // go to middle
                  List<Double> values = pathCommands.CalcSwerveValues(
                      drivetrain.getPose(), targetPoses.get(2)); // change 2
                  DriveCommands.joystickDrive(
                      drivetrain, () -> values.get(0), () -> values.get(1), () -> values.get(2));
                })
            .until(() -> pathCommands.CloseEnough(drivetrain.getPose(), targetPoses.get(2))),
        Commands.run(() -> {
              // run intake for x seconds
            })
            .withTimeout(5)
        // next: go back thru to AZ, using reverse of previous poses!!!! / or maybe using next poses
        // bc you go the other way?
        // idk its an opportunity for more differeniation *CRIES*

        );
  }
  //
  // only for aligning from in front of hub start, add differentiation! <---------
  /*public Command ShootThenClimbAuto(DrivetrainSubsystem drivetrain, Boolean fieldRelative, Double m_period,
  Robot robot, Double targetYaw) {
      List<Pose2d> targetPoses = Arrays.asList(
          new Pose2d(1.678, 3.75, Rotation2d.fromDegrees(0)),
          new Pose2d(1.438, 3.745, Rotation2d.fromDegrees(-180))
      );
      return Commands.sequence(
          Commands.run(() -> { // align to tag
              PathCommands.AlignToTag(drivetrain, m_period, fieldRelative);
          }).until(() -> (RobotContainer.targetRange <= 2 && RobotContainer.targetYaw <= 5)),
          //
          Commands.run(() -> { // align to front of hub from center start point <- DIFFERENTIATE!!!
                  List<Double> values = PathCommands.CalcSwerveValues(drivetrain.getPose(), targetPoses.get(0));
                  PathCommands.setSwerve(drivetrain, m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
          }).until(() -> PathCommands.CloseEnough(drivetrain.getPose(),targetPoses.get(0))),
          //
          Commands.runOnce(() -> { // shoots all fuel (runs both shooter for 5 sec, maybe add sensor input idk)
              //SubsystemCommands.RunLeftShooter();
              //SubsystemCommands.RunRightShooter();
          }).withTimeout(5),
          //
          Commands.run(() -> { // drives to tower
                  List<Double> values = PathCommands.CalcSwerveValues(drivetrain.getPose(), targetPoses.get(1));
                  PathCommands.setSwerve(drivetrain, m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
          }).until(() -> PathCommands.CloseEnough(drivetrain.getPose(),targetPoses.get(0)))
          // next put climbing code // should withtimeout ones be run not runOnce?
      );
  }*/
}
