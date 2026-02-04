package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.States.State;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.mechanisms.MechanismBase;

public abstract class AbstractSubsystemBase extends edu.wpi.first.wpilibj2.command.SubsystemBase {
  @Override
  public abstract void periodic();

  public abstract void setMechGoals(State[] goal_States, int[] usedProfiles, int[] usedFeedbacks);

  public abstract void setMechGoals(State... goal_States);

  public abstract State[] getMechStates();

  public abstract MechanismBase<?>[] getMechanisms();

  public abstract void setMechGoal(
      int mechanism, State goal_State, int usedProfile, int usedFeedback);

  public abstract void setMechGoal(int mechanism, State goal_State);

  public abstract Command getMechSysIdCommand(int mechanism, SysIdType type);

  public abstract State getMechState(int mechanism);

  public abstract MechanismBase<?> getMechanism(int mechanism);
}
