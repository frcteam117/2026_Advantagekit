package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.States.AngularPV_State;
import frc.robot.util.States.State;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.mechanisms.MechanismBase;

public class Intake extends SubsystemBase {
  public final MechanismBase<AngularPV_State> arm;
  public final MechanismBase<AngularPV_State> roller;

  /** Constructor for the ShooterSubsystem. */
  public Intake() {
    arm = new MechanismBase<>(IntakeConstants.armConfig, this);
    roller = new MechanismBase<>(IntakeConstants.rollerConfig, this);
  }

  @Override
  public void periodic() {
    arm.update();
    roller.update();
  }

  public void setMechGoals(State[] goal_States, int[] usedProfiles, int[] usedFeedbacks) {
    arm.setGoal(goal_States[0], usedProfiles[0], usedFeedbacks[0]);
    roller.setGoal(goal_States[1], usedProfiles[1], usedFeedbacks[1]);
  }

  public void setMechGoals(State... goal_States) {
    arm.setGoal(goal_States[0]);
    roller.setGoal(goal_States[1]);
  }

  public Command getSysIdCommand(int mechanism, SysIdType type) {
    return mechanism == 0 ? arm.getSysIdCommand(type) : roller.getSysIdCommand(type);
  }

  public State[] getMechStates() {
    return new State[] {arm.getState(), roller.getState()};
  }

  public Pose3d[] getMechPose3ds() {
    return null;
  }
}
