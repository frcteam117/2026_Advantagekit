package frc.robot.subsystems.indexer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemBase2Mech;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.mechanisms.MechanismBase;
import frc.robot.util.states.State;
import frc.robot.util.states.bases.PosVel_State;

public class IndexerSubsystem extends SubsystemBase2Mech<PosVel_State, PosVel_State> {
  public IndexerSubsystem() {
    super(IndexerConstants.HOPPER_CONFIG, IndexerConstants.KICKER_CONFIG);
  }

  public Pose3d[] getPose3ds() {
    return new Pose3d[0];
  }

  // Hopper

  public void setHopperGoal(State goal_State, int usedProfile, int usedFeedback) {
    setMech0Goal(goal_State, usedProfile, usedFeedback);
  }

  public void setHopperGoal(State goal_State) {
    setMech0Goal(goal_State);
  }

  public Command getHopperSysIdCommand(SysIdType type) {
    return getMech0SysIdCommand(type);
  }

  public PosVel_State getHopperState() {
    return getMech0State();
  }

  public MechanismBase<PosVel_State> getHopper() {
    return getMechanism0();
  }

  // Kicker

  public void setKickerGoal(State goal_State, int usedProfile, int usedFeedback) {
    setMech1Goal(goal_State, usedProfile, usedFeedback);
  }

  public void setKickerGoal(State goal_State) {
    setMech1Goal(goal_State);
  }

  public Command getKickerSysIdCommand(SysIdType type) {
    return getMech1SysIdCommand(type);
  }

  public PosVel_State getKickerState() {
    return getMech1State();
  }

  public MechanismBase<PosVel_State> getKicker() {
    return getMechanism1();
  }
}
