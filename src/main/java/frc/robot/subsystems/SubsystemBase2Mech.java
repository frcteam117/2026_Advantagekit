package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.mechanisms.MechanismBase;
import frc.robot.util.mechanisms.MechanismBase.MechanismConfig;
import frc.robot.util.states.State;

public class SubsystemBase2Mech<Output0_State extends State, Output1_State extends State>
    extends AbstractSubsystemBase {
  private final MechanismBase<Output0_State> mechanism0;
  private final MechanismBase<Output1_State> mechanism1;

  public SubsystemBase2Mech(
      MechanismConfig<Output0_State> mechanism0Config,
      MechanismConfig<Output1_State> mechanism1Config) {
    mechanism0 = new MechanismBase<>(mechanism0Config, this);
    mechanism1 = new MechanismBase<>(mechanism1Config, this);
  }

  @Override
  public void periodic() {
    mechanism0.update();
    mechanism1.update();
  }

  // Grouped mechanism methods

  @Override
  public void setMechGoals(State[] goal_States, int[] usedProfiles, int[] usedFeedbacks) {
    mechanism0.setGoal(goal_States[0], usedProfiles[0], usedFeedbacks[0]);
    mechanism1.setGoal(goal_States[1], usedProfiles[1], usedFeedbacks[1]);
  }

  @Override
  public void setMechGoals(State... goal_States) {
    mechanism0.setGoal(goal_States[0]);
    mechanism1.setGoal(goal_States[1]);
  }

  @Override
  public State[] getMechStates() {
    return new State[] {mechanism0.getState(), mechanism1.getState()};
  }

  @Override
  public MechanismBase<?>[] getMechanisms() {
    return new MechanismBase<?>[] {mechanism0, mechanism1};
  }

  // Individual mechanism methods

  @Override
  public void setMechGoal(int mechanism, State goal_State, int usedProfile, int usedFeedback) {
    switch (mechanism) {
      case 0: {
        mechanism0.setGoal(goal_State, usedProfile, usedFeedback);
        return;
      }
      case 1: {
        mechanism1.setGoal(goal_State, usedProfile, usedFeedback);
        return;
      }
    }
  }

  @Override
  public void setMechGoal(int mechanism, State goal_State) {
    switch (mechanism) {
      case 0: {
        mechanism0.setGoal(goal_State);
        return;
      }
      case 1: {
        mechanism1.setGoal(goal_State);
        return;
      }
    }
  }

  @Override
  public Command getMechSysIdCommand(int mechanism, SysIdType type) {
    switch (mechanism) {
      case 0:
        return mechanism0.getSysIdCommand(type);
      case 1:
        return mechanism1.getSysIdCommand(type);
    }
    return Commands.none();
  }

  @Override
  public State getMechState(int mechanism) {
    switch (mechanism) {
      case 0:
        return mechanism0.getState();
      case 1:
        return mechanism1.getState();
    }
    return null;
  }

  @Override
  public MechanismBase<?> getMechanism(int mechanism) {
    switch (mechanism) {
      case 0:
        return mechanism0;
      case 1:
        return mechanism1;
    }
    return null;
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

  // Mechanism 1

  protected void setMech1Goal(State goal_State, int usedProfile, int usedFeedback) {
    mechanism1.setGoal(goal_State, usedProfile, usedFeedback);
  }

  protected void setMech1Goal(State goal_State) {
    mechanism1.setGoal(goal_State);
  }

  protected Command getMech1SysIdCommand(SysIdType type) {
    return mechanism1.getSysIdCommand(type);
  }

  protected Output1_State getMech1State() {
    return mechanism1.getState();
  }

  protected MechanismBase<Output1_State> getMechanism1() {
    return mechanism1;
  }
}
