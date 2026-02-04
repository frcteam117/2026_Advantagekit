package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemBase2Mech;
import frc.robot.util.States.AngularPV_State;
import frc.robot.util.States.State;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.mechanisms.MechanismBase;

public class Shooter extends SubsystemBase2Mech<AngularPV_State, AngularPV_State> {

  public Shooter() {
    super(ShooterConstants.hoodConfig, ShooterConstants.flywheelConfig);
  }

  public Pose3d[] getPose3ds() {
    return new Pose3d[] {
      new Pose3d(-0.24286, 0, 0.58996, new Rotation3d(0, -getHoodState().rad(), 0))
    };
  }

  // Hood

  public void setHoodGoal(State goal_State, int usedProfile, int usedFeedback) {
    setMech0Goal(goal_State, usedProfile, usedFeedback);
  }

  public void setHoodGoal(State goal_State) {
    setMech0Goal(goal_State);
  }

  public Command getHoodSysIdCommand(SysIdType type) {
    return getMech0SysIdCommand(type);
  }

  public AngularPV_State getHoodState() {
    return getMech0State();
  }

  public MechanismBase<AngularPV_State> getHood() {
    return getMechanism0();
  }

  // Flywheel

  public void setFlywheelGoal(State goal_State, int usedProfile, int usedFeedback) {
    setMech1Goal(goal_State, usedProfile, usedFeedback);
  }

  public void setFlywheelGoal(State goal_State) {
    setMech1Goal(goal_State);
  }

  public Command getFlywheelSysIdCommand(SysIdType type) {
    return getMech1SysIdCommand(type);
  }

  public AngularPV_State getFlywheelState() {
    return getMech1State();
  }

  public MechanismBase<AngularPV_State> getFlywheel() {
    return getMechanism1();
  }
}
