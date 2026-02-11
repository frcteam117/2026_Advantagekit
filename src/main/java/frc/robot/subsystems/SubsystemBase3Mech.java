package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.StateUtil.State;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.mechanisms.MechanismBase;
import frc.robot.util.mechanisms.MechanismBase.MechanismConfig;

public class SubsystemBase3Mech<
        Output0_State extends State, Output1_State extends State, Output2_State extends State>
    extends AbstractSubsystemBase {
  private final MechanismBase<Output0_State> mechanism0;
  private final MechanismBase<Output1_State> mechanism1;
  private final MechanismBase<Output2_State> mechanism2;

  public SubsystemBase3Mech(
      MechanismConfig<Output0_State> mechanism0Config,
      MechanismConfig<Output1_State> mechanism1Config,
      MechanismConfig<Output2_State> mechanism2Config) {
    mechanism0 = new MechanismBase<>(mechanism0Config, this);
    mechanism1 = new MechanismBase<>(mechanism1Config, this);
    mechanism2 = new MechanismBase<>(mechanism2Config, this);
  }

  @Override
  public void periodic() {
    mechanism0.update();
    mechanism1.update();
    mechanism2.update();
  }

  // Grouped mechanism methods

  @Override
  public void setMechGoals(State[] goal_States, int[] usedProfiles, int[] usedFeedbacks) {
    mechanism0.setGoal(goal_States[0], usedProfiles[0], usedFeedbacks[0]);
    mechanism1.setGoal(goal_States[1], usedProfiles[1], usedFeedbacks[1]);
    mechanism2.setGoal(goal_States[2], usedProfiles[2], usedFeedbacks[2]);
  }

  @Override
  public void setMechGoals(State... goal_States) {
    mechanism0.setGoal(goal_States[0]);
    mechanism1.setGoal(goal_States[1]);
    mechanism2.setGoal(goal_States[2]);
  }

  @Override
  public State[] getMechStates() {
    return new State[] {mechanism0.getState(), mechanism1.getState(), mechanism2.getState()};
  }

  @Override
  public MechanismBase<?>[] getMechanisms() {
    return new MechanismBase<?>[] {mechanism0, mechanism1, mechanism2};
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
      case 2: {
        mechanism2.setGoal(goal_State, usedProfile, usedFeedback);
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
      case 2: {
        mechanism2.setGoal(goal_State);
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
      case 2:
        return mechanism2.getSysIdCommand(type);
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
      case 2:
        return mechanism2.getState();
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
      case 2:
        return mechanism2;
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

  protected MechanismBase<Output0_State> getMech0() {
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

  protected MechanismBase<Output1_State> getMech1() {
    return mechanism1;
  }

  // Mechanism 2

  protected void setMech2Goal(State goal_State, int usedProfile, int usedFeedback) {
    mechanism2.setGoal(goal_State, usedProfile, usedFeedback);
  }

  protected void setMech2Goal(State goal_State) {
    mechanism2.setGoal(goal_State);
  }

  protected Command getMech2SysIdCommand(SysIdType type) {
    return mechanism2.getSysIdCommand(type);
  }

  protected Output2_State getMech2State() {
    return mechanism2.getState();
  }

  protected MechanismBase<Output2_State> getMech2() {
    return mechanism2;
  }
}
