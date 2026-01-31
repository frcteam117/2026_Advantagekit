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

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.logging.LogUtil;
import frc.robot.util.logging.TunableDouble;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Initialize URCL
    Logger.registerURCL(URCL.startExternal());

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  private Pose3d[] getIntakePose(double rad) {
    // math based on https://www.desmos.com/calculator/8e3rmxmen2
    double g = Math.sqrt(0.081730601 - 0.0338354902642 * Math.cos(rad));
    double ifVar = Math.PI < rad % (2 * Math.PI) ? 1 : -1;
    double theta_a = rad - 0.564035611052;
    double theta_b = 2.55195984092
        - Math.acos((-0.0732526549 - (g * g)) / (-0.558793904763 * g))
        + ifVar * Math.acos((0.0743979216 - (g * g)) / (-0.121100614367 * g));
    double theta_e = 3.8131545094
        + theta_a
        + Math.acos((0.0732526549 - (g * g)) / (-0.138708357355 * g))
        - ifVar * Math.acos((-0.0743979216 - (g * g)) / (-0.558799646743 * g));
    // these are of b_1 - a_1 on the graph
    double x = 0.06006
        + (0.253999673228 * Math.cos(0.462274273714 + theta_b))
        - (0.217012684652 * Math.cos(0.471540446498 + theta_a));
    double y = -0.00769
        + (0.253999673228 * Math.sin(0.462274273714 + theta_b))
        - (0.217012684652 * Math.sin(0.471540446498 + theta_a));
    double h = Math.sqrt(x * x + y * y);
    double atanOfh = Math.atan2(y, x);
    double theta_c =
        atanOfh + 1.22843881587 - Math.acos((-0.0107190074 - h * h) / (-0.510740385715 * h));
    double theta_d =
        atanOfh + 4.7931822977 + Math.acos((0.0107190074 - h * h) / (-0.466882974631 * h));

    Pose3d[] poses = new Pose3d[5];
    // the thetas are negative because pitch rotates down from z to x and the thetas are the angle
    // above the normal position
    poses[0] = new Pose3d(0.16543, 0, 0.18708, new Rotation3d(0, -theta_a, 0));
    poses[1] = new Pose3d(0.22549, 0, 0.17939, new Rotation3d(0, -theta_b, 0));
    poses[2] = new Pose3d(
        new Translation3d(0.19333, 0, 0.09858)
            .rotateBy(poses[0].getRotation())
            .plus(poses[0].getTranslation()),
        new Rotation3d(0, -theta_c, 0));
    poses[3] = new Pose3d(
        new Translation3d(0.22734, 0, 0.11328)
            .rotateBy(poses[1].getRotation())
            .plus(poses[1].getTranslation()),
        new Rotation3d(0, -theta_d, 0));
    poses[4] = new Pose3d(
        new Translation3d(0.25318, 0, 0.11817)
            .rotateBy(poses[0].getRotation())
            .plus(poses[0].getTranslation()),
        new Rotation3d(0, -theta_e, 0));

    return poses;
  }

  TunableDouble intakeMin_rad = new TunableDouble("IntakeMin_rad", 0.5, () -> true);
  TunableDouble intakeMax_rad = new TunableDouble("IntakeMax_rad", 2.1, () -> true);
  TunableDouble hoodMin_rad = new TunableDouble("HoodMin_rad", 0, () -> true);
  TunableDouble hoodMax_rad = new TunableDouble("HoodMax_rad", Math.PI / 4, () -> true);
  TunableDouble cyclePeriod_s = new TunableDouble("cyclePeriod_s", 2, () -> true);
  double cycle = 0;

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    Logger.recordOutput("testPose", new Pose2d(1, 1, new Rotation2d(0)));
    cycle += 2 * Math.PI * Constants.Robot.codePeriod_s / cyclePeriod_s.getAsDouble();

    Pose3d[] poses = getIntakePose(intakeMin_rad.getAsDouble()
        + (intakeMax_rad.getAsDouble() - intakeMin_rad.getAsDouble()) * (1 + Math.cos(cycle)) / 2);
    Logger.recordOutput("testComponentPoses", new Pose3d[] {
      poses[0],
      poses[1],
      poses[2],
      poses[3],
      poses[4],
      new Pose3d(
          -0.24286,
          0,
          0.58996,
          new Rotation3d(
              0,
              -(hoodMin_rad.getAsDouble()
                  + (hoodMax_rad.getAsDouble() - hoodMin_rad.getAsDouble())
                      * (1 + Math.cos(cycle))
                      / 2),
              0))
    });
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    LogUtil.getInstance().runUpdateMethods();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    robotContainer.resetSimulationField();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    SimulatedArena.getInstance()
        .addGamePieceProjectile(ReefscapeCoralOnFly.DropFromCoralStation(
            ReefscapeCoralOnFly.CoralStationsSide.LEFT_STATION, DriverStation.Alliance.Red, true));
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    robotContainer.updateSimulation();
  }
}
