package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.States.AngularPV_State;
import frc.robot.util.States.State;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.mechanisms.MechanismBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public final MechanismBase<AngularPV_State> hood;
  public final MechanismBase<AngularPV_State> flywheel;

  /** Constructor for the ShooterSubsystem. */
  public Shooter() {
    hood = new MechanismBase<>(ShooterConstants.hoodConfig, this);
    flywheel = new MechanismBase<>(ShooterConstants.flywheelConfig, this);
  }

  @Override
  public void periodic() {
    hood.update();
    flywheel.update();
    Logger.recordOutput(ShooterConstants.logName + "/Pose3d", new Pose3d[] {
      new Pose3d(0, 0, -1, Rotation3d.kZero),
      new Pose3d(0, 0, -1, Rotation3d.kZero),
      new Pose3d(
          0.0,
          0.0,
          .3 + flywheel.getState().vel() / 200,
          new Rotation3d(0, -hood.getState().rad(), 0))
    });
  }

  public void setMechGoals(State[] goal_States, int[] usedProfiles, int[] usedFeedbacks) {
    hood.setGoal(goal_States[0], usedProfiles[0], usedFeedbacks[0]);
    flywheel.setGoal(goal_States[1], usedProfiles[1], usedFeedbacks[1]);
  }

  public void setMechGoals(State... goal_States) {
    hood.setGoal(goal_States[0]);
    flywheel.setGoal(goal_States[1]);
  }

  public Command getSysIdCommand(int mechanism, SysIdType type) {
    return mechanism == 0 ? hood.getSysIdCommand(type) : flywheel.getSysIdCommand(type);
  }

  public State[] getMechStates() {
    return new State[] {hood.getState(), flywheel.getState()};
  }

  public Pose3d[] getMechPose3ds() {
    return null;
  }
}
