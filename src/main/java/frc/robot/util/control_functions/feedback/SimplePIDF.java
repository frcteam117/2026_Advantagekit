package frc.robot.util.control_functions.feedback;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.util.States.PosVel_State;
import frc.robot.util.States.Voltage_State;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.logging.LogUtil;

public class SimplePIDF extends ControlFunctionBase {
  private String name = "SimplePIDF";
  private final PIDController pid;
  private final SimpleMotorFeedforward ff;
  private PosVel_State lastNext_State;
  private boolean enableTuning = false;

  public SimplePIDF(
      PIDController pid,
      SimpleMotorFeedforward ff,
      PosVel_State startingState,
      String mechanismTuningLogName,
      String name) {
    this(pid, ff, startingState, mechanismTuningLogName);
    this.name = name;
  }

  public SimplePIDF(
      PIDController pid,
      SimpleMotorFeedforward ff,
      PosVel_State startingState,
      String mechanismTuningLogName) {
    this.pid = pid;
    this.ff = ff;
    lastNext_State = startingState;
    LogUtil.createTunablePID(mechanismTuningLogName + "/" + name + "/", pid, () -> enableTuning);
    LogUtil.createTunableFF(mechanismTuningLogName + "/" + name + "/", ff, () -> enableTuning);
  }

  public Voltage_State calculate(PosVel_State next_State, PosVel_State mechanism_State) {
    Voltage_State voltage =
        new Voltage_State(ff.calculateWithVelocities(lastNext_State.vel(), next_State.vel())
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
