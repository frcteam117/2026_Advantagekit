package frc.robot.util.control_functions.feedback;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.logging.LogUtil;
import frc.robot.util.mechanisms.MechanismConstants;
import frc.robot.util.states.bases.PosVel_State;
import frc.robot.util.states.premade.Voltage_State;

public class SimplePIDF extends ControlFunctionBase {
  private String name = "SimplePIDF";
  private final PIDController pid;
  private final SimpleMotorFeedforward ff;
  private PosVel_State lastNext_State;
  private boolean enableTuning = false;

  public SimplePIDF(MechanismConstants<?> config) {
    this(config, "SimplePIDF");
  }

  public SimplePIDF(MechanismConstants<?> config, String profileName) {
    name = profileName;
    if (config.pid == null) {
      // TODO: make this an exception
    }
    if (config.simpleFF == null) {
      // TODO: make this an exception
    }
    if (!PosVel_State.class.isAssignableFrom(config.start_State.getClass())) {
      // TODO: make this an exception
    }
    pid = config.pid;
    ff = config.simpleFF;
    lastNext_State = (PosVel_State) config.start_State;
    LogUtil.createTunablePID(config.tuningLogName + "/" + name, pid, () -> enableTuning);
    LogUtil.createTunableFF(config.tuningLogName + "/" + name, ff, () -> enableTuning);
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
