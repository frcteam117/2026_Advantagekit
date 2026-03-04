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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.RobotCommands;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.indexer.IndexerCommands;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedCommands;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
// import frc.robot.subsystems.shooter.ShooterIO;
// import frc.robot.subsystems.shooter.ShooterIOReal;
// import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.*;
import frc.robot.util.SysIdUtil;
import frc.robot.util.SysIdUtil.SysIdType;
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

        vision = new VisionSubsystem(drivetrain::accept, new VisionIOPhotonVision(1));
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

        vision = new VisionSubsystem(
            drivetrain::accept,
            new VisionIOPhotonVisionSim(0, driveSimulation::getSimulatedDriveTrainPose));
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
        // vision = null;
        vision = new VisionSubsystem(drivetrain::accept, new VisionIO() {});
        break;
    }
    intake = new IntakeSubsystem();
    indexer = new IndexerSubsystem();
    shooter = new ShooterSubsystem();

    SysIdUtil.registerController(controller);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DrivetrainCommands.wheelRadiusCharacterization(drivetrain));
    autoChooser.addOption(
        "Drive SysId (QuasistaticForward)",
        DrivetrainCommands.getDriveSysId(drivetrain, SysIdType.QuasistaticForward));
    autoChooser.addOption(
        "Drive SysId (QuasistaticReverse)",
        DrivetrainCommands.getDriveSysId(drivetrain, SysIdType.QuasistaticReverse));
    autoChooser.addOption(
        "Drive SysId (DynamicForward)",
        DrivetrainCommands.getDriveSysId(drivetrain, SysIdType.DynamicForward));
    autoChooser.addOption(
        "Drive SysId (DynamicReverse)",
        DrivetrainCommands.getDriveSysId(drivetrain, SysIdType.DynamicReverse));
    autoChooser.addOption(
        "Azimuth SysId (QuasistaticForward)",
        DrivetrainCommands.getAzimuthSysId(drivetrain, SysIdType.QuasistaticForward));
    autoChooser.addOption(
        "Azimuth SysId (QuasistaticReverse)",
        DrivetrainCommands.getAzimuthSysId(drivetrain, SysIdType.QuasistaticReverse));
    autoChooser.addOption(
        "Azimuth SysId (DynamicForward)",
        DrivetrainCommands.getAzimuthSysId(drivetrain, SysIdType.DynamicForward));
    autoChooser.addOption(
        "Azimuth SysId (DynamicReverse)",
        DrivetrainCommands.getAzimuthSysId(drivetrain, SysIdType.DynamicReverse));
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

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    LedCommands.ledCommand(() -> ShooterCommands.isAutoAimReady(shooter));
    // Default command, normal field-relative drive
    drivetrain.setDefaultCommand(DrivetrainCommands.joystickDrive(
        drivetrain,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> controller.getRightX()));

    controller
        .R2()
        .whileTrue(RobotCommands.hubAutoShoot(drivetrain, shooter, indexer, controller.triangle()));
    controller2
        .R1()
        .whileTrue(RobotCommands.hubAutoShoot(drivetrain, shooter, indexer, controller.triangle()));
    controller.L2().whileTrue(IntakeCommands.RunRollerForwardForTuning(intake));

    indexer.setDefaultCommand(IndexerCommands.stop(indexer));
    controller.L1().whileTrue(IndexerCommands.runForwardCommand(indexer));
    controller.R1().whileTrue(IndexerCommands.runBackwardCommand(indexer));

    shooter.setDefaultCommand(ShooterCommands.stop(shooter));
    controller.triangle().whileTrue(ShooterCommands.runForward(shooter));
    // zzzzzzzzzzzzzzzzzzzzz
    controller.circle().whileTrue(IntakeCommands.RunRollerForwardForTuning(intake));
    controller.cross().whileTrue(IntakeCommands.RunRollerBackwardForTuning(intake));
    intake.setDefaultCommand(IntakeCommands.stopCommand(intake));
    controller.square().onTrue(IntakeCommands.stopCommand(intake));

    controller.povUp().whileTrue(ShooterCommands.raiseHood(shooter));
    controller.povDown().whileTrue(ShooterCommands.lowerHood(shooter));
    controller
        .povLeft()
        .whileTrue(RobotCommands.hubAutoShoot(drivetrain, shooter, indexer, controller.triangle()));
    //
    // controller.R2().whileTrue(IndexerCommands.runKickerForwardForTuning)
    // controller.circle().whileFalse(IntakeCommands.(intake));

    // Reset gyro / odometry
    final Runnable resetGyro = RobotConstants.currentMode == RobotConstants.Mode.SIM
        ? () -> drivetrain.resetOdometry(
            driveSimulation
                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
        // simulation
        : () -> drivetrain.resetNavX(); // zero gyro
    controller.touchpad().onTrue(Commands.runOnce(resetGyro, drivetrain).ignoringDisable(true));
    // controller
    //     .button(1)
    //     .onTrue(Commands.runOnce(() -> {})
    //         .finallyDo(() -> CommandScheduler.getInstance().schedule(getAutonomousCommand())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
    // return Commands.parallel(
    //         Commands.waitSeconds(3).andThen(IndexerCommands.runForwardCommand(indexer)),
    //         RobotCommands.hubAutoShoot(drivetrain, shooter, indexer, controller.triangle()))
    //     .withTimeout(8);
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
