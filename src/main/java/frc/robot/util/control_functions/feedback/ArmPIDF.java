package frc.robot.util.control_functions.feedback;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.States.PosVel_State;
import frc.robot.util.States.Voltage_State;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.logging.LogUtil;

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

  public Voltage_State calculate(PosVel_State next_State, PosVel_State mechanism_State) {
    Voltage_State voltage = new Voltage_State(ff.calculateWithVelocities(
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
