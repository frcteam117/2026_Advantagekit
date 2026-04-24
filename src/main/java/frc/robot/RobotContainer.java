// Copyright 2021-2024 FRC 6328
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

package frc.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.RobotCommands;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.indexer.IndexerCommands;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterSubsystem;
// import frc.robot.subsystems.shooter.ShooterIO;
// import frc.robot.subsystems.shooter.ShooterIOReal;
// import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.*;
import frc.robot.util.SysIdUtil;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DrivetrainSubsystem drivetrain;
  private final VisionSubsystem vision;
  private final IntakeSubsystem intake;
  private final IndexerSubsystem indexer;
  public final ShooterSubsystem shooter;
  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);
  private final CommandPS5Controller controller2 = new CommandPS5Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems and commands. */
  public RobotContainer() {
    switch (RobotConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain = new DrivetrainSubsystem(
            new GyroIONavX(),
            new ModuleIONova(0),
            new ModuleIONova(1),
            new ModuleIONova(2),
            new ModuleIONova(3),
            (pose) -> {});
        intake = new IntakeSubsystem(new IntakeIOReal());
        shooter = new ShooterSubsystem(new ShooterIOReal());

        vision = new VisionSubsystem(
            drivetrain::accept, new VisionIOPhotonVision(0), new VisionIOPhotonVision(1));
        break;
      case SIM:
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation = new SwerveDriveSimulation(
            DrivetrainConstants.Chassis.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        // Sim robot, instantiate physics sim IO implementations
        drivetrain = new DrivetrainSubsystem(
            new GyroIOSim(driveSimulation.getGyroSimulation()),
            new ModuleIOSim(driveSimulation.getModules()[0]),
            new ModuleIOSim(driveSimulation.getModules()[1]),
            new ModuleIOSim(driveSimulation.getModules()[2]),
            new ModuleIOSim(driveSimulation.getModules()[3]),
            driveSimulation::setSimulationWorldPose);
        intake = new IntakeSubsystem(new IntakeIO() {});
        shooter = new ShooterSubsystem(new ShooterIO() {});

        vision = new VisionSubsystem(
            drivetrain::accept,
            new VisionIOPhotonVisionSim(0, driveSimulation::getSimulatedDriveTrainPose),
            new VisionIOPhotonVisionSim(1, driveSimulation::getSimulatedDriveTrainPose));
        break;
      default:
        // Replayed robot, disable IO implementations
        drivetrain = new DrivetrainSubsystem(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            (pose) -> {});
        intake = new IntakeSubsystem(new IntakeIO() {});
        shooter = new ShooterSubsystem(new ShooterIO() {});
        // vision = null;
        vision = new VisionSubsystem(drivetrain::accept, new VisionIO() {}, new VisionIO() {});
        break;
    }
    indexer = new IndexerSubsystem();

    SysIdUtil.registerController(controller);

    NamedCommands.registerCommand(
        "IntakeDeploy",
        IntakeCommands.intakeFuel(intake, () -> false, () -> true)
            .alongWith(IndexerCommands.intakingAgitation(indexer)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf)));
    NamedCommands.registerCommand("PivotDown", IntakeCommands.lowerIntake(intake));
    NamedCommands.registerCommand("AutoOverBump", DrivetrainCommands.pathOverBump(drivetrain));
    NamedCommands.registerCommand(
        "AutoOverBumpWithFlywheelRev",
        DrivetrainCommands.pathOverBump(drivetrain)
            .deadlineWith(ShooterCommands.runForward(shooter)));
    // NamedCommands.registerCommand("ResetPose", DrivetrainSubsystem.resetPoseWithVision());
    NamedCommands.registerCommand(
        "alignAndShoot",
        Commands.parallel(
            IntakeCommands.defaultCommand(intake, () -> false),
            RobotCommands.autoAim(
                drivetrain,
                shooter,
                indexer,
                () -> DrivetrainCommands.pivotBasedCenterOfRotation(intake.getPivotPos()),
                () -> true)));
    NamedCommands.registerCommand("IntakeRollerOn", Commands.none());
    NamedCommands.registerCommand("IntakeRollerOff", Commands.none());
    NamedCommands.registerCommand(
        "stopFlywheel",
        Commands.parallel(IndexerCommands.stop(indexer), ShooterCommands.stopAndZeroHood(shooter)));
    NamedCommands.registerCommand(
        "flywheel_acc_1", RobotCommands.autoAimRevFlywheels(drivetrain::getPose, shooter));
    NamedCommands.registerCommand("flywheel_acc_2", ShooterCommands.runForward(shooter));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DrivetrainCommands.wheelRadiusCharacterization(drivetrain));
    autoChooser.addOption(
        "Stationary Preload",
        RobotCommands.autoAim(
                drivetrain,
                shooter,
                indexer,
                () -> DrivetrainCommands.pivotBasedCenterOfRotation(intake.getPivotPos()),
                () -> true)
            .withTimeout(10));
    // autoChooser.addOption(
    //     "Drive SysId (QuasistaticForward)",
    //     DrivetrainCommands.getLinearDriveSysId(drivetrain, SysIdType.QuasistaticForward));
    // autoChooser.addOption(
    //     "Drive SysId (QuasistaticReverse)",
    //     DrivetrainCommands.getLinearDriveSysId(drivetrain, SysIdType.QuasistaticReverse));
    // autoChooser.addOption(
    //     "Drive SysId (DynamicForward)",
    //     DrivetrainCommands.getLinearDriveSysId(drivetrain, SysIdType.DynamicForward));
    // autoChooser.addOption(
    //     "Drive SysId (DynamicReverse)",
    //     DrivetrainCommands.getLinearDriveSysId(drivetrain, SysIdType.DynamicReverse));
    // autoChooser.addOption(
    //     "Azimuth SysId (QuasistaticForward)",
    //     DrivetrainCommands.getAzimuthSysId(drivetrain, SysIdType.QuasistaticForward));
    // autoChooser.addOption(
    //     "Azimuth SysId (QuasistaticReverse)",
    //     DrivetrainCommands.getAzimuthSysId(drivetrain, SysIdType.QuasistaticReverse));
    // autoChooser.addOption(
    //     "Azimuth SysId (DynamicForward)",
    //     DrivetrainCommands.getAzimuthSysId(drivetrain, SysIdType.DynamicForward));
    // autoChooser.addOption(
    //     "Azimuth SysId (DynamicReverse)",
    //     DrivetrainCommands.getAzimuthSysId(drivetrain, SysIdType.DynamicReverse));
    // autoChooser.addOption("Albert Auto", AutoBuilder.buildAuto("pidaytoppathfull"));
    // autoChooser.addOption("Kai Auto", AutoBuilder.buildAuto("kai auto 1"));

    // autoChooser.addOption(
    //     "Flywheel SysId (Quasistatic)",
    //     ShooterCommands.flywheelSysId(shooter, SysIdType.Quasistatic));
    // autoChooser.addOption(
    //     "Flywheel SysId (Dynamic)", ShooterCommands.flywheelSysId(shooter, SysIdType.Dynamic));
    // autoChooser.addOption(
    //     "Hood SysId (Quasistatic)", ShooterCommands.hoodSysId(shooter, SysIdType.Quasistatic));
    // autoChooser.addOption(
    //     "Hood SysId (Dynamic)", ShooterCommands.hoodSysId(shooter, SysIdType.Dynamic));

    // Configure the button bindings
    configureButtonBindings();
  }

  // private Rotation2d prevTarget = Rotation2d.kZero;
  private List<Double> shooterTuningDistances = new ArrayList<>();
  private List<Double> shooterTuningRIOVels = new ArrayList<>();
  private List<Double> shooterTuningPDHVels = new ArrayList<>();
  private List<Double> shooterTuningFlywheelVels = new ArrayList<>();
  private List<Double> shooterTuningHoodPoses = new ArrayList<>();

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    PathPlannerLogging.setLogActivePathCallback(activePath ->
        Logger.recordOutput("PathPlanner/ActivePath", activePath.stream().toArray(Pose2d[]::new)));
    PathPlannerLogging.setLogCurrentPoseCallback(
        currentPose -> Logger.recordOutput("PathPlanner/CurrentPose", currentPose));
    PathPlannerLogging.setLogTargetPoseCallback(
        targetPose -> Logger.recordOutput("PathPlanner/TargetPose", targetPose));
    drivetrain.setDefaultCommand(DrivetrainCommands.joystickDriveAtAngle(
        drivetrain,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightY(),
        () -> -controller.getRightX(),
        .2,
        controller.R3(),
        controller.R1(),
        () -> new Translation2d(),
        () -> false));
    controller.L3().whileTrue(DrivetrainCommands.pathOverBump(drivetrain));
    controller
        .cross()
        .whileTrue(RobotCommands.faceHubAndDrive(
            drivetrain,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            // () -> DrivetrainCommands.pivotBasedCenterOfRotation(intake.getPivotPos())
            () -> new Translation2d(-.2, 0)));

    // Intake
    intake.setDefaultCommand(IntakeCommands.defaultCommand(intake, controller.L1()));
    controller
        .circle()
        .whileTrue(IntakeCommands.outtakeFuel(intake, controller.L1())
            .alongWith(IndexerCommands.runBackwardCommand(indexer)));
    controller
        .L2()
        .whileTrue(IntakeCommands.intakeFuel(intake, controller.L1(), () -> false))
        .whileTrue(IndexerCommands.intakingAgitation(indexer)
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    controller
        .button(10)
        .whileTrue(Commands.run(() -> intake.setPivotGoalPos(Radians.of(-1.5)), intake));
    controller
        .button(9)
        .whileTrue(Commands.run(() -> intake.setPivotGoalPos(Radians.of(-0.7)), intake));

    // Indexer
    indexer.setDefaultCommand(IndexerCommands.stop(indexer));
    if (ShooterCommands.isTuning) {
      controller2
          .R1()
          .whileTrue(
              // ShooterCommands.autoAim(shooter, drivetrain::getPose, null, null, null));
              Commands.parallel(
                  IndexerCommands.runForwardCommand(indexer),
                  Commands.run(() -> IntakeCommands.shooting = true)
                      .finallyDo(() -> IntakeCommands.shooting = false)));
    }
    controller
        .square()
        .whileTrue(Commands.startEnd(
            () -> IndexerCommands.runningBackwards = true,
            () -> IndexerCommands.runningBackwards = false));
    controller2.square().whileTrue(IndexerCommands.runForwardCommand(indexer));
    controller2.circle().whileTrue(IndexerCommands.runBackwardCommand(indexer));

    // Shooter
    if (ShooterCommands.isTuning) {
      shooter.setDefaultCommand(ShooterCommands.stopAndHoldHood(shooter));
    } else {
      shooter.setDefaultCommand(ShooterCommands.stopAndZeroHood(shooter));
    }
    controller.triangle().whileTrue(RobotCommands.setPointRevThenShoot(shooter, indexer));
    controller2.povUp().whileTrue(ShooterCommands.raiseHood(shooter));
    controller2.povDown().whileTrue(ShooterCommands.lowerHood(shooter));
    controller2.triangle().whileTrue(ShooterCommands.runForward(shooter));

    // new button bindings:
    controller2.L1().whileTrue(RobotCommands.autoAimRevFlywheels(drivetrain::getPose, shooter));
    // should this be whileTrue? vvv
    controller
        .R2()
        .whileTrue(Commands.runOnce(() -> drivetrain.resetPoseWithVision())
            .andThen(RobotCommands.autoAim(
                drivetrain,
                shooter,
                indexer,
                () -> DrivetrainCommands.pivotBasedCenterOfRotation(intake.getPivotPos()),
                () -> true))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    if (ShooterCommands.isTuning) {
      controller2
          .R2()
          .whileTrue(RobotCommands.autoFaceTarget(
              drivetrain,
              () -> DrivetrainCommands.pivotBasedCenterOfRotation(intake.getPivotPos())));
    }
    // make separate shooting and facing hub commands once max figures out his commands stuff ^^^
    // controller.triangle().whileTrue(IndexerCommands.runBackwardCommand(indexer));
    // controller
    //     .cross()
    //     .whileTrue(RobotCommands.faceHubAndDrive(
    //         drivetrain,
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         () -> DrivetrainCommands.pivotBasedCenterOfRotation(
    //             intake.getPivotPos()))); // make aimToPass

    controller2.button(13).debounce(1).onTrue(Commands.runOnce(() -> {
      final double distance = RobotCommands.getTarget(drivetrain.getPose())
          .getDistance(drivetrain.getPose().getTranslation());
      ShooterCommands.setAutoAimPoint(distance);
      shooterTuningDistances.add(distance);
      shooterTuningRIOVels.add(shooter.getRIOFlywheelVel().in(RadiansPerSecond));
      shooterTuningPDHVels.add(shooter.getPDHFlywheelVel().in(RadiansPerSecond));
      shooterTuningFlywheelVels.add(ShooterCommands.getFlywheelGoal());
      shooterTuningHoodPoses.add(shooter.getHoodPos().in(Radians));
      Logger.recordOutput(
          "ShooterTuning/Poses",
          Arrays.stream(shooterTuningDistances.toArray(Double[]::new))
              .mapToDouble(value -> value)
              .toArray());
      Logger.recordOutput(
          "ShooterTuning/RIOVels",
          Arrays.stream(shooterTuningRIOVels.toArray(Double[]::new))
              .mapToDouble(value -> value)
              .toArray());
      Logger.recordOutput(
          "ShooterTuning/PDHVels",
          Arrays.stream(shooterTuningPDHVels.toArray(Double[]::new))
              .mapToDouble(value -> value)
              .toArray());
      Logger.recordOutput(
          "ShooterTuning/HoodPoses",
          Arrays.stream(shooterTuningHoodPoses.toArray(Double[]::new))
              .mapToDouble(value -> value)
              .toArray());
    }));

    // snake mode around middle
    // controller
    //     .triangle()
    //     .whileTrue(DrivetrainCommands.joystickDriveAtAngle(
    //         drivetrain,
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         .05,
    //         () -> DrivetrainCommands.pivotBasedCenterOfRotation(
    //             intake.getPivotState().pos()),
    //         () -> false));

    // snake mode around intake
    // controller
    //     .triangle()
    //     .whileTrue(DrivetrainCommands.joystickDriveAtAngle(
    //         drivetrain,
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         .05,
    //         controller.R3(),
    //         controller.R1(),
    //         () -> new Translation2d(
    //             (DrivetrainConstants.Chassis.bumperLength_m / 2) + Units.inchesToMeters(2), 0),
    //         () -> false));

    // angle from right joystick mode
    // controller
    //     .triangle()
    //     .whileTrue(DrivetrainCommands.joystickDriveAtAngle(
    //         drivetrain,
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         () -> Rotation2d.fromRadians(
    //             Math.atan2(-controller.getRightX(), -controller.getRightY())),
    //         () -> DrivetrainCommands.pivotBasedCenterOfRotation(
    //             intake.getPivotState().pos()),
    //         () -> false));

    // Reset gyro / odometry
    // final Runnable resetGyro = RobotConstants.currentMode == RobotConstants.Mode.SIM
    //     ? () -> drivetrain.resetOdometry(
    //         driveSimulation
    //             .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
    //     // simulation
    //     : () -> drivetrain.resetPoseWithVision(); // zero gyro
    // controller
    //     .touchpad()
    //     .onTrue(
    //         RobotBase.isReal()
    //             ? DrivetrainCommands.replacePoseWithVision(drivetrain)
    //             : Commands.runOnce(resetGyro, drivetrain).ignoringDisable(true));
    controller
        .touchpad()
        .onTrue(Commands.runOnce(() -> drivetrain.resetPoseWithVision()).ignoringDisable(true));
    // controller
    //     .L3()
    //     .onTrue(Commands.runOnce(() -> drivetrain.resetPoseWithVision()).ignoringDisable(true));
    controller2
        .touchpad()
        .onTrue(Commands.runOnce(() -> drivetrain.resetPoseWithVision()).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
    // try {
    //   return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Example Path"));
    // } catch (FileVersionException | IOException | ParseException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    // return null;
  }

  public void resetSimulationField() {
    if (RobotConstants.currentMode != RobotConstants.Mode.SIM) return;

    drivetrain.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (RobotConstants.currentMode != RobotConstants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "_FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "_FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }

  //   @AutoLogOutput(key = "0_Supersystem/ComponentPoses")
  //   private Pose3d[] getSupersystemPose3ds() {
  //     Pose3d[] intakePoses = intake.getPose3ds();
  //     return new Pose3d[] {
  //       intakePoses[0],
  //       intakePoses[1],
  //       intakePoses[2],
  //       intakePoses[3],
  //       intakePoses[4],
  //       new Pose3d(-0.24286, 0, 0.58996, new Rotation3d(0, -shooter.getHoodState().pos(), 0))
  //     };
  //   }
}
