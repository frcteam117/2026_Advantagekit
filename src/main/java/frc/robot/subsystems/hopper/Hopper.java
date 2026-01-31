package frc.robot.subsystems.hopper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.States.AngularPV_State;
import frc.robot.util.States.State;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.mechanisms.MechanismBase;

public class Hopper extends SubsystemBase {
  public final MechanismBase<AngularPV_State> roller;

  /** Constructor for the ShooterSubsystem. */
  public Hopper() {
    roller = new MechanismBase<>(HopperConstants.rollerConfig, this);
  }

  @Override
  public void periodic() {
    roller.update();
  }

  public void setMechGoals(State[] goal_States, int[] usedProfiles, int[] usedFeedbacks) {
    roller.setGoal(goal_States[0], usedProfiles[0], usedFeedbacks[0]);
  }

  public void setMechGoals(State... goal_States) {
    roller.setGoal(goal_States[0]);
  }

  public Command getSysIdCommand(int mechanism, SysIdType type) {
    return roller.getSysIdCommand(type);
  }

  public State[] getMechStates() {
    return new State[] {roller.getState()};
  }

  public Pose3d[] getMechPose3ds() {
    return null;
  }
}
