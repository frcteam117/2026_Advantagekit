package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemBase2Mech;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.mechanisms.MechanismBase;
import frc.robot.util.states.State;
import frc.robot.util.states.bases.PosVel_State;

public class ShooterSubsystem extends SubsystemBase2Mech<PosVel_State, PosVel_State> {

  public ShooterSubsystem() {
    super(ShooterConstants.HOOD_CONFIG, ShooterConstants.FLYWHEEL_CONFIG);
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

  public MechanismBase<PosVel_State> getHood() {
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

  public PosVel_State getFlywheelState() {
    return getMech1State();
  }

  public MechanismBase<PosVel_State> getFlywheel() {
    return getMechanism1();
  }
}
