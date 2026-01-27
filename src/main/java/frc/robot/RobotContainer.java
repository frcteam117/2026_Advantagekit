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

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.*;
import frc.robot.util.SysIdUtil.SysIdType;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
            new GyroIONavX(),
            new ModuleIONova(0),
            new ModuleIONova(1),
            new ModuleIONova(2),
            new ModuleIONova(3),
            (pose) -> {});

        this.vision = new Vision(
            drive,
            new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
            new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        this.shooter = new Shooter(new ShooterIOReal());
        break;
      case SIM:
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation = new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Body body = new Body();
        // BodyFixture fixture = new BodyFixture(new Circle(2));
        // fixture.setSensor(true);
        // body.addFixture(fixture);
        // body.setTransform(new Transform());
        // World simWorld = new World<>();
        // simWorld.addCollisionListener(new CollisionListenerAdapter<>().collision(body.)));

        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIOSim(driveSimulation.getGyroSimulation()),
            new ModuleIOSim(driveSimulation.getModules()[0]),
            new ModuleIOSim(driveSimulation.getModules()[1]),
            new ModuleIOSim(driveSimulation.getModules()[2]),
            new ModuleIOSim(driveSimulation.getModules()[3]),
            driveSimulation::setSimulationWorldPose);

        vision = new Vision(
            drive,
            new VisionIOPhotonVisionSim(
                camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
            new VisionIOPhotonVisionSim(
                camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

        shooter = new Shooter(new ShooterIOSim());

        break;
      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            (pose) -> {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

        shooter = new Shooter(new ShooterIO() {});

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        ShooterCommands.flywheelSysId(shooter, SysIdType.QuasistaticForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        ShooterCommands.flywheelSysId(shooter, SysIdType.QuasistaticReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)",
        ShooterCommands.flywheelSysId(shooter, SysIdType.DynamicForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)",
        ShooterCommands.flywheelSysId(shooter, SysIdType.DynamicReverse));
    autoChooser.addOption(
        "Hood SysId (Quasistatic)", ShooterCommands.hoodSysId(shooter, SysIdType.Quasistatic));
    autoChooser.addOption(
        "Hood SysId (Dynamic)", ShooterCommands.hoodSysId(shooter, SysIdType.Dynamic));
    autoChooser.addOption(
        "Turret SysId (Quasistatic Forward)",
        ShooterCommands.turretSysId(shooter, SysIdType.QuasistaticForward));
    autoChooser.addOption(
        "Turret SysId (Quasistatic Reverse)",
        ShooterCommands.turretSysId(shooter, SysIdType.QuasistaticReverse));
    autoChooser.addOption(
        "Turret SysId (Dynamic Forward)",
        ShooterCommands.turretSysId(shooter, SysIdType.DynamicForward));
    autoChooser.addOption(
        "Turret SysId (Dynamic Reverse)",
        ShooterCommands.turretSysId(shooter, SysIdType.DynamicReverse));

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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(DriveCommands.joystickDrive(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRawAxis(2)));
    shooter.setDefaultCommand(ShooterCommands.trackPosition(
        shooter, drive::getPose, () -> new Translation2d(4.489, 4.026)));
    // shooter.setDefaultCommand(ShooterCommands.runFlywheelGoalVelocity(
    //     shooter, () -> -500 + 1000 * (-controller.getRawAxis(3) + 1) / 2));

    // Lock to 0° when A button is held
    // controller
    //         .button(0)
    //         .whileTrue(DriveCommands.joystickDriveAtAngle(
    //                 drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> new
    // Rotation2d()));

    // Switch to X pattern when X button is pressed
    // controller.button(1).onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro / odometry
    final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
        ? () -> drive.resetOdometry(
            driveSimulation
                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
        // simulation
        : () -> drive.resetOdometry(
            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    controller.button(1).onTrue(Commands.runOnce(() -> {}).finallyDo(() -> DriveCommands.pathToReef(
            drive::getPose, controller.button(1)::getAsBoolean)
        .schedule()));
    // Example Coral Placement Code
    // TODO: delete these code for your own project
    if (Constants.currentMode == Constants.Mode.SIM) {
      // L4 placement
      controller.button(3).onTrue(Commands.runOnce(() -> SimulatedArena.getInstance()
          .addGamePieceProjectile(new ReefscapeCoralOnFly(
              driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
              new Translation2d(0.4, 0),
              driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
              driveSimulation.getSimulatedDriveTrainPose().getRotation(),
              Meters.of(2),
              MetersPerSecond.of(1.5),
              Degrees.of(-80)))));
      // L3 placement
      controller.button(2).onTrue(Commands.runOnce(() -> SimulatedArena.getInstance()
          .addGamePieceProjectile(new ReefscapeCoralOnFly(
              driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
              new Translation2d(0.4, 0),
              driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
              driveSimulation.getSimulatedDriveTrainPose().getRotation(),
              Meters.of(1.35),
              MetersPerSecond.of(1.5),
              Degrees.of(-60)))));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
