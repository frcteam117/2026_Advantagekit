package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.mechanisms.MechanismBase;
import frc.robot.util.mechanisms.MechanismBase.MechanismConfig;
import frc.robot.util.states.State;

public class SubsystemBase1Mech<Output0_State extends State> extends AbstractSubsystemBase {
  private final MechanismBase<Output0_State> mechanism0;

  public SubsystemBase1Mech(MechanismConfig<Output0_State> mechanismConfigs) {
    mechanism0 = new MechanismBase<>(mechanismConfigs, this);
  }

  @Override
  public void periodic() {
    mechanism0.update();
  }

  // Grouped mechanism methods

  @Override
  public void setMechGoals(State[] goal_States, int[] usedProfiles, int[] usedFeedbacks) {
    mechanism0.setGoal(goal_States[0], usedProfiles[0], usedFeedbacks[0]);
  }

  public void setMechGoals(State goal_State, int usedProfile, int usedFeedback) {
    mechanism0.setGoal(goal_State, usedProfile, usedFeedback);
  }

  @Override
  public void setMechGoals(State... goal_States) {
    mechanism0.setGoal(goal_States[0]);
  }

  @Override
  public State[] getMechStates() {
    return new State[] {mechanism0.getState()};
  }

  @Override
  public MechanismBase<?>[] getMechanisms() {
    return new MechanismBase[] {mechanism0};
  }

  // Individual mechanism methods

  @Override
  public void setMechGoal(int mechanism, State goal_State, int usedProfile, int usedFeedback) {
    mechanism0.setGoal(goal_State, usedProfile, usedFeedback);
  }

  @Override
  public void setMechGoal(int mechanism, State goal_State) {
    mechanism0.setGoal(goal_State);
  }

  @Override
  public Command getMechSysIdCommand(int mechanism, SysIdType type) {
    return mechanism0.getSysIdCommand(type);
  }

  @Override
  public Output0_State getMechState(int mechanism) {
    return mechanism0.getState();
  }

  @Override
  public MechanismBase<Output0_State> getMechanism(int mechanism) {
    return mechanism0;
  }

  // Mechanism 0

  protected void setMech0Goal(State goal_State, int usedProfile, int usedFeedback) {
    mechanism0.setGoal(goal_State, usedProfile, usedFeedback);
  }

  protected void setMech0Goal(State goal_State) {
    mechanism0.setGoal(goal_State);
  }

  protected Command getMech0SysIdCommand(SysIdType type) {
    return mechanism0.getSysIdCommand(type);
  }

  protected Output0_State getMech0State() {
    return mechanism0.getState();
  }

  protected MechanismBase<Output0_State> getMechanism0() {
    return mechanism0;
  }
}
