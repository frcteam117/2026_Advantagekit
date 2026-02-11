package frc.robot.util.control_functions.feedback;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.StateUtil.AngularP_State;
import frc.robot.util.StateUtil.PosVel_State;
import frc.robot.util.StateUtil.Volt_State;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.logging.LogUtil;
import frc.robot.util.mechanisms.MechanismConstants;

public class ArmPIDF extends ControlFunctionBase {
  private String name = "ArmPIDF";
  private final PIDController pid;
  private final ArmFeedforward ff;
  private final double cmOffset_rad;
  private PosVel_State lastNext_State;
  private boolean enableTuning = false;

  public ArmPIDF(
      PIDController pid,
      ArmFeedforward ff,
      PosVel_State startingState,
      double cmOffset_rad,
      String mechanismTuningLogName,
      String name) {
    this(pid, ff, startingState, cmOffset_rad, mechanismTuningLogName);
    this.name = name;
  }

  public ArmPIDF(
      PIDController pid,
      ArmFeedforward ff,
      PosVel_State startingState,
      double cmOffset_rad,
      String mechanismTuningLogName) {
    this.pid = pid;
    this.ff = ff;
    this.cmOffset_rad = cmOffset_rad;
    lastNext_State = startingState;
    LogUtil.createTunablePID(mechanismTuningLogName + "/" + name + "/", pid, () -> enableTuning);
    LogUtil.createTunableFF(mechanismTuningLogName + "/" + name + "/", ff, () -> enableTuning);
  }

  public ArmPIDF(MechanismConstants config) {
    this(config, "ArmPIDF");
  }

  public ArmPIDF(MechanismConstants config, String profileName) {
    name = profileName;
    if (config.pid == null) {
      // TODO: make this an exception
    }
    if (config.armFF == null) {
      // TODO: make this an exception
    }
    if (config.cmOffset_Pos == null) {
      config.cmOffset_Pos = new AngularP_State(0);
    }
    if (!PosVel_State.class.isAssignableFrom(config.start_State.getClass())) {
      // TODO: make this an exception
    }
    pid = config.pid;
    ff = config.armFF;
    lastNext_State = (PosVel_State) config.start_State;
    cmOffset_rad = config.cmOffset_Pos.pos();
    LogUtil.createTunablePID(config.tuningLogName + "/" + name + "/", pid, () -> enableTuning);
    LogUtil.createTunableFF(config.tuningLogName + "/" + name + "/", ff, () -> enableTuning);
  }

  public Volt_State calculate(PosVel_State next_State, PosVel_State mechanism_State) {
    Volt_State voltage = new Volt_State(ff.calculateWithVelocities(
            mechanism_State.pos() + cmOffset_rad, lastNext_State.vel(), next_State.vel())
        + pid.calculate(mechanism_State.pos(), lastNext_State.pos()));
    lastNext_State = next_State;
    return voltage;
  }

  public void updateState(PosVel_State mechanism_State) {
    lastNext_State = mechanism_State;
  }

  public void resetState(PosVel_State mechanism_State) {
    lastNext_State = mechanism_State;
  }

  @Override
  public String getControlFunctionName() {
    return name;
  }

  @Override
  public void setTunable(boolean isTunable) {
    enableTuning = isTunable;
  }
}
