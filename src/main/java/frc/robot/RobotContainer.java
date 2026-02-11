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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
// import frc.robot.subsystems.shooter.ShooterIO;
// import frc.robot.subsystems.shooter.ShooterIOReal;
// import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.*;
import frc.robot.util.StateUtil.AngularP_State;
import frc.robot.util.StateUtil.AngularV_State;
import frc.robot.util.SysIdUtil;
import frc.robot.util.SysIdUtil.SysIdType;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
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
  private final DrivetrainSubsystem drive;
  private final Vision vision;
  public final IntakeSubsystem intake;
  public final IndexerSubsystem indexer;
  public final ShooterSubsystem shooter;
  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RobotConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new DrivetrainSubsystem(
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
        break;
      case SIM:
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation = new SwerveDriveSimulation(
            DrivetrainConstants.Chassis.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
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
        drive = new DrivetrainSubsystem(
            new GyroIOSim(driveSimulation.getGyroSimulation()),
            new ModuleIOSim(driveSimulation.getModules()[0]),
            new ModuleIOSim(driveSimulation.getModules()[1]),
            new ModuleIOSim(driveSimulation.getModules()[2]),
            new ModuleIOSim(driveSimulation.getModules()[3]),
            driveSimulation::setSimulationWorldPose);

        vision = new Vision(
            drive,
            new VisionIOPhotonVisionSim(
                VisionConstants.camera0Name,
                VisionConstants.robotToCamera0,
                driveSimulation::getSimulatedDriveTrainPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.camera1Name,
                VisionConstants.robotToCamera1,
                driveSimulation::getSimulatedDriveTrainPose));
        break;
      default:
        // Replayed robot, disable IO implementations
        drive = new DrivetrainSubsystem(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            (pose) -> {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
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
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption("Drive SysId (Quasistatic)", drive.getDriveSysId(SysIdType.Quasistatic));
    autoChooser.addOption("Drive SysId (Dynamic)", drive.getDriveSysId(SysIdType.Dynamic));
    autoChooser.addOption("Turn SysId (Quasistatic)", drive.getAzimuthSysId(SysIdType.Quasistatic));
    autoChooser.addOption("Turn SysId (Dynamic)", drive.getAzimuthSysId(SysIdType.Dynamic));
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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(DriveCommands.joystickDrive(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRawAxis(2)));
    shooter.setDefaultCommand(Commands.run(
        () -> shooter.setMechGoals(new AngularP_State(0), new AngularV_State(0)), shooter));
    controller
        .button(1)
        .onTrue(Commands.run(() -> shooter.setHoodGoal(new AngularP_State(.25)), shooter));
    controller
        .button(2)
        .onTrue(Commands.run(() -> shooter.setHoodGoal(new AngularP_State(0.5)), shooter));
    controller
        .button(3)
        .onTrue(Commands.run(() -> shooter.setHoodGoal(new AngularP_State(0.75)), shooter));

    controller
        .button(4)
        .onTrue(Commands.run(() -> shooter.setFlywheelGoal(new AngularV_State(200)), shooter));
    controller
        .button(5)
        .onTrue(Commands.run(() -> shooter.setFlywheelGoal(new AngularV_State(400)), shooter));
    controller
        .button(6)
        .onTrue(Commands.run(() -> shooter.setFlywheelGoal(new AngularV_State(600)), shooter));

    // Reset gyro / odometry
    final Runnable resetGyro = RobotConstants.currentMode == RobotConstants.Mode.SIM
        ? () -> drive.resetOdometry(
            driveSimulation
                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
        // simulation
        : () -> drive.resetOdometry(
            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
    controller
        .button(1)
        .onTrue(Commands.runOnce(() -> {})
            .finallyDo(() -> CommandScheduler.getInstance().schedule(getAutonomousCommand())));
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
    if (RobotConstants.currentMode != RobotConstants.Mode.SIM) return;

    drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (RobotConstants.currentMode != RobotConstants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    // Logger.recordOutput("controller/button0", controller.button(0).getAsBoolean());
    // Logger.recordOutput("controller/button1", controller.button(1).getAsBoolean());
    // Logger.recordOutput("controller/button2", controller.button(2).getAsBoolean());
    // Logger.recordOutput("controller/button3", controller.button(3).getAsBoolean());
    Logger.recordOutput(
        "_FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "_FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }

  @AutoLogOutput(key = "0_Supersystem/ComponentPoses")
  private Pose3d[] getSupersystemPose3ds() {
    Pose3d[] intakePoses = intake.getPose3ds();
    return new Pose3d[] {
      intakePoses[0],
      intakePoses[1],
      intakePoses[2],
      intakePoses[3],
      intakePoses[4],
      new Pose3d(-0.24286, 0, 0.58996, new Rotation3d(0, -shooter.getHoodState().pos(), 0))
    };
  }
}
