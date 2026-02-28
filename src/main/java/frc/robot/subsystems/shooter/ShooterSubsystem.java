package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemBase3Mech;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.states.State;
import frc.robot.util.states.bases.PosVel_State;

public class ShooterSubsystem extends SubsystemBase3Mech<PosVel_State, PosVel_State, PosVel_State> {

  public ShooterSubsystem() {
    super(
        ShooterConstants.HOOD_CONFIG,
        ShooterConstants.RIO_FLYWHEEL_CONFIG,
        ShooterConstants.PDH_FLYWHEEL_CONFIG);
  }

  public Pose3d getPose3d() {
    return new Pose3d(-0.24286, 0, 0.58996, new Rotation3d(0, -getHoodState().pos(Radians), 0));
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

  public PosVel_State getHoodState() {
    return getMech0State();
  }

  // RIO Flywheel

  public void setRIOFlywheelGoal(State goal_State, int usedProfile, int usedFeedback) {
    setMech1Goal(goal_State, usedProfile, usedFeedback);
  }

  public void setRIOFlywheelGoal(State goal_State) {
    setMech1Goal(goal_State);
  }

  public Command getRIOFlywheelSysIdCommand(SysIdType type) {
    return getMech1SysIdCommand(type);
  }

  public PosVel_State getRIOFlywheelState() {
    return getMech1State();
  }

  // PDH Flywheel

  public void setPDHFlywheelGoal(State goal_State, int usedProfile, int usedFeedback) {
    setMech2Goal(goal_State, usedProfile, usedFeedback);
  }

  public void setPDHFlywheelGoal(State goal_State) {
    setMech2Goal(goal_State);
  }

  public Command getPDHFlywheelSysIdCommand(SysIdType type) {
    return getMech2SysIdCommand(type);
  }

  public PosVel_State getPDHFlywheelState() {
    return getMech2State();
  }
}
