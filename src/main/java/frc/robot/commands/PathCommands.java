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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//
import edu.wpi.first.wpilibj.Timer;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.time.Period;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PathCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  //
  //private final SubsystemCommands
  //
  double drivetrain_xSpeed = 0.0;
  double drivetrain_ySpeed =  0.0;
  double drivetrain_rot = 0.0;
  boolean drivetrain_fieldRelative = true;

  public static double limiter = 1; // adjust!!
  public static double speedCap = 0.5; // adjust!!
  private static final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
  private static final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
  private static final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(9); // import these from robot for better continuity?
  Timer timer;
  

  public record drivetrainValues(double xSpeed, double ySpeed,double rot, boolean fieldRelative) {};

    String allianceColor = "blue";
    static Boolean alliancePresent = false;
    Optional<Alliance> alliance = DriverStation.getAlliance();

    public PathCommands() {
        if (alliance.isPresent()) {
            alliancePresent = true;
        } else {
        }
    }


    //Drivetrain m_swerve; // does this just work????????
    //
    //
    static AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    //
    //====
    public Double getDistanceFromHub(DrivetrainSubsystem drivetrain) {
        double hubX = 0;
        double hubY = 0;
        if (alliancePresent) {
            if (alliance.get() == Alliance.Blue) {
                hubX = 4.666; // ADD CONSTANTS FILE WITH ALL THIS DATA LATER
                hubY = 4;
            }
            else if (alliance.get() == Alliance.Red) {
                hubX = 12; // ADD CONSTANTS FILE WITH ALL THIS DATA LATER
                hubY = 4;
            }
        }
        //Pose2d AllianceHubPosition = 
        double xDistance = Math.abs(drivetrain.getPose().getX()-hubX);
        double yDistance = Math.abs(drivetrain.getPose().getY()-hubY);
        double distance = Math.sqrt((xDistance*xDistance)+(yDistance*yDistance)); // PYTHAGOREANS THEOREMMM (tho he def
        //- didnt discover it firsttttt)
        return distance;
    }


    //
    public void AlignToTag(DrivetrainSubsystem drivetrain, VisionSubsystem vision, PhotonCamera camera2, Double m_period, Boolean fieldRelative) {
        //    change 2??? vvv
        var cameraData = vision.getCameraResults(camera2);
            double targetRange = cameraData.targetRange();
            boolean targetVisible = cameraData.targetVisible();
            double targetYaw = cameraData.targetYaw();
            double kPVision_Turn = cameraData.kPVision_Turn();
            SmartDashboard.putString("cameraData",cameraData.toString()); // if targetRange > 2
            if (true && targetVisible) { // reset the camera photonvision values so the targetrange stuff can be accurate?
                SmartDashboard.putNumber("check #",2);
                double rot = -1.0 * targetYaw * kPVision_Turn * 5;//(SwerveConstants.TOP_SPEED_METERS_PER_SEC/0.6);//SwerveConstants.kMaxAngularSpeed;
                // DONT USE 5.
                double xSpeed = 0; // change from 0 for both???
                    ///-m_xspeedLimiter.calculate(MathUtil.applyDeadband(targetRange * 0.5, 0.03)) // CONFIGURE STUFF SO U CAN TEST IF TS WORKS W/ SWERVE!!!!!
                    //* SwerveConstants.TOP_SPEED_METERS_PER_SEC;
                double ySpeed = 0;
                    //-m_yspeedLimiter.calculate(MathUtil.applyDeadband(targetYaw * kPVision_Turn, 0.03))
                   // * //SwerveConstants.TOP_SPEED_METERS_PER_SEC;
                    //SmartDashboard.putBoolean("setSwerve",true);
                    setSwerve(m_period, xSpeed, ySpeed, rot, fieldRelative);
            }
    }

    //
    public void setSwerve( double m_period, double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        double a =
        m_xspeedLimiter.calculate(MathUtil.applyDeadband(xSpeed, 0.03))
            * SwerveConstants.TOP_SPEED_METERS_PER_SEC
            * 0.4;
    double b =
        m_yspeedLimiter.calculate(MathUtil.applyDeadband(ySpeed, 0.03))
            * SwerveConstants.TOP_SPEED_METERS_PER_SEC
            * 0.4;
    double c =
        m_rotLimiter.calculate(MathUtil.applyDeadband(rot, 0.04))
            * 1.4;

        System.out.println("aligning to tag");
        System.out.println("setting swerve to drive");
        System.out.println(a+" "+b+" "+c+" "+fieldRelative+" "+m_period);
       // RobotContainer.getDrivetrain().drive(a, b, c, fieldRelative, m_period);
        drivetrain_xSpeed=a;
        drivetrain_ySpeed=b;
        drivetrain_rot=c;
        drivetrain_fieldRelative=fieldRelative;

    }

    public drivetrainValues getDrivetrainValues() {
        return new drivetrainValues(drivetrain_xSpeed, drivetrain_ySpeed, 
        drivetrain_rot, drivetrain_fieldRelative);
    }



    public void TestDrive()
    {

        //RobotContainer.getDrivetrain().drive(1.0, 0, 0, true,0.02);
    }

    public boolean CloseEnough(Pose2d curPose, Pose2d targetPose) { // gotta be a better way 2 do this but again idfk
        double difX = targetPose.getX()-curPose.getX(); 
        double difY = targetPose.getY()-curPose.getY();
        System.out.println(difX);
        System.out.println(difY);
        if ((Math.abs(difX)+Math.abs(difY))/2 <= 0.05) { // adjust 0.05!!!
            double difRot = 
                targetPose.getRotation().getDegrees()-curPose.getRotation().getDegrees(); 
            System.out.println(difRot);
            if (difRot <= 1) {
                return true;
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }
    }

    public List<Double> CalcSwerveValues(Pose2d curPose, Pose2d targetPose) {
        double difX = targetPose.getX()-curPose.getX(); 
        double difY = targetPose.getY()-curPose.getY(); 
        double difRot = targetPose.getRotation().getDegrees()-curPose.getRotation().getDegrees();
        //
        //
        double xSpeed = 0;
        double ySpeed = 0;
        double rot = 0; // add this later idk man
        //Rotation2d difRot = Pose2d.getRotation();
        xSpeed = difX * limiter;
        ySpeed = difY * limiter;
        rot = difRot;
        if (Math.abs(xSpeed) > speedCap) {
            if (xSpeed >= 0) {
                xSpeed = speedCap;
            }
            else {
                xSpeed = -speedCap;
            }
        }
        if (Math.abs(ySpeed) > speedCap) {
            if (ySpeed >= 0) {
                ySpeed = speedCap;
            }
            else {
                ySpeed = -speedCap;
            }
        }
        if (difX <= 0.01) {
            xSpeed = 0;
        }
        if (difY <= 0.01) {
            ySpeed = 0;
        }
        //if (rot > ) add rot limiter? idrk?
        // is this math right?????
        List<Double> values = Arrays.asList(xSpeed,ySpeed,rot);
        return values;

    }

    public Command BlankCommand() {
        return Commands.runOnce( () -> {});
    }
    public Command StopSwerve(DrivetrainSubsystem drivetrain, Boolean fieldRelative, Double m_period) {
        //Drivetrain m_swerve,
        return DriveCommands.joystickDrive(drivetrain,()->0.0, ()->0.0, ()->0.0);
                //drivetrain.drive(0.0, 0.0, 0.0, fieldRelative, m_period); // add way to stop the robot?????
        //});

    }
//===========================================================================================================
    // IDK IF I HAVE TO ADD .relativeTo TO THE END OF ALL THE POSE OR NOT??????????????????

    public Command Path1Command(DrivetrainSubsystem drivetrain, Boolean fieldRelative, Double m_period) {        
        Pose2d targetPose = new Pose2d(-0.5, 0.5, Rotation2d.fromDegrees(0));
        System.out.println("thingy run");
        return Commands.sequence(
            Commands.run(() -> {
                    System.out.println("Path1Command run");
                    System.out.println(drivetrain.getPose());
                    System.out.println(targetPose);
                    List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPose);
                    setSwerve( m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> CloseEnough(drivetrain.getPose(),targetPose))
        );
    }

    public Command Path2Command(DrivetrainSubsystem drivetrain, Boolean fieldRelative, Double m_period) {
        Pose2d targetPose = new Pose2d(0.5, -0.5, Rotation2d.fromDegrees(0));
        return Commands.sequence(
            Commands.run(() -> {
                    List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPose);
                    setSwerve( m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> CloseEnough(drivetrain.getPose(),targetPose))
        );
    }
    // change to be from current pose VVV
    public Command DriveToCenterFromOrigin(DrivetrainSubsystem drivetrain, Boolean fieldRelative, Double m_period, 
    Robot robot, Double targetYaw) {
        List<Pose2d> targetPoses = Arrays.asList(new Pose2d(
            RobotContainer.AprilTagPoses.get(12).getX(), // go to a tag
            RobotContainer.AprilTagPoses.get(12).getY(),
            Rotation2d.fromDegrees(targetYaw) //does this need to be the difference of smth? idk
        ),
        new Pose2d(6.5, 0.6, Rotation2d.fromDegrees(0)),
        new Pose2d(8.3, 4, Rotation2d.fromDegrees(0))
        );        
        return Commands.sequence(
            Commands.run(() -> {
                    List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPoses.get(0));
                    setSwerve( m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> CloseEnough(drivetrain.getPose(),targetPoses.get(0))),
            //
            Commands.run(() -> {
                List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPoses.get(1));
                setSwerve( m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> CloseEnough(drivetrain.getPose(),targetPoses.get(1))),

            Commands.run(() -> {
                List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPoses.get(2));
                setSwerve( m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> CloseEnough(drivetrain.getPose(),targetPoses.get(2)))
        );
    }

    
}