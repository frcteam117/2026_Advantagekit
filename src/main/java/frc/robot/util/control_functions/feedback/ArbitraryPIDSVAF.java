package frc.robot.util.control_functions.feedback;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.logging.LogUtil;
import frc.robot.util.mechanisms.MechanismConstants;
import frc.robot.util.states.bases.PosVel_State;
import frc.robot.util.states.premade.Voltage_State;

public class ArbitraryPIDSVAF extends ControlFunctionBase {
  private String name = "ArbitraryPIDSVAF";
  private final PIDController pid;
  private final SimpleMotorFeedforward ff;
  private final InterpolatingDoubleTreeMap arbitraryFF;
  private final double cmOffset_rad;
  private PosVel_State lastNext_State;
  private boolean enableTuning = false;

  public ArbitraryPIDSVAF(MechanismConstants<?> config) {
    this(config, "ArmPIDF");
  }

  public ArbitraryPIDSVAF(MechanismConstants<?> config, String profileName) {
    name = profileName;
    if (config.pid == null) {
      // TODO: make this an exception
    }
    if (config.simpleFF == null) {
      // TODO: make this an exception
    }
    if (config.arbitraryFF == null) {
      // TODO: make this an exception
    }
    if (config.cmOffset_rad == null) {
      config.cmOffset_rad = 0d;
    }
    if (!PosVel_State.class.isAssignableFrom(config.start_State.getClass())) {
      // TODO: make this an exception
    }
    pid = config.pid;
    ff = config.simpleFF;
    arbitraryFF = config.arbitraryFF;
    lastNext_State = (PosVel_State) config.start_State;
    cmOffset_rad = config.cmOffset_rad;
    LogUtil.createTunablePID(config.tuningLogName + "/" + name, pid, () -> enableTuning);
    LogUtil.createTunableFF(config.tuningLogName + "/" + name, ff, () -> enableTuning);
  }

  public Voltage_State calculate(PosVel_State next_State, PosVel_State mechanism_State) {
    Voltage_State voltage = new Voltage_State(
        ff.calculateWithVelocities(lastNext_State.vel(RadiansPerSecond), next_State.vel(Radians))
            + arbitraryFF.get(mechanism_State.pos())
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
