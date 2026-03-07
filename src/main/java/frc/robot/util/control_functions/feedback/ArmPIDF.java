package frc.robot.util.control_functions.feedback;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.logging.LogUtil;
import frc.robot.util.mechanisms.MechanismConstants;
import frc.robot.util.states.bases.PosVel_State;
import frc.robot.util.states.premade.Voltage_State;

public class ArmPIDF extends ControlFunctionBase {
  private String name = "ArmPIDF";
  private final PIDController pid;
  private final ArmFeedforward ff;
  private final double cmOffset_rad;
  private PosVel_State lastNext_State;
  private boolean enableTuning = false;

  public ArmPIDF(MechanismConstants<?> config) {
    this(config, "ArmPIDF");
  }

  public ArmPIDF(MechanismConstants<?> config, String profileName) {
    name = profileName;
    if (config.pid == null) {
      // TODO: make this an exception
    }
    if (config.armFF == null) {
      // TODO: make this an exception
    }
    if (config.cmOffset_rad == null) {
      config.cmOffset_rad = 0d;
    }
    if (!PosVel_State.class.isAssignableFrom(config.start_State.getClass())) {
      // TODO: make this an exception
    }
    pid = config.pid;
    ff = config.armFF;
    lastNext_State = (PosVel_State) config.start_State;
    cmOffset_rad = config.cmOffset_rad;
    LogUtil.createTunablePID(config.tuningLogName + "/" + name, pid, () -> enableTuning);
    LogUtil.createTunableFF(config.tuningLogName + "/" + name, ff, () -> enableTuning);
  }

  public Voltage_State calculate(PosVel_State next_State, PosVel_State mechanism_State) {
    Voltage_State voltage = new Voltage_State(ff.calculateWithVelocities(
            mechanism_State.pos(Radians) + cmOffset_rad,
            lastNext_State.vel(RadiansPerSecond),
            next_State.vel(RadiansPerSecond))
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
