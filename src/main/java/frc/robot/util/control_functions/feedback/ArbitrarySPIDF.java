package frc.robot.util.control_functions.feedback;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.logging.LogUtil;
import frc.robot.util.logging.TunableDouble;
import frc.robot.util.mechanisms.MechanismConstants;
import frc.robot.util.states.bases.PosVel_State;
import frc.robot.util.states.premade.Voltage_State;

public class ArbitrarySPIDF extends ControlFunctionBase {
  private String name = "ArbitrarySPIDF";
  private final PIDController pid;
  private final SimpleMotorFeedforward ff;
  private final InterpolatingDoubleTreeMap arbitraryForwardS;
  private final InterpolatingDoubleTreeMap arbitraryReverseS;
  private double minVel;
  private double maxPosError;
  private PosVel_State lastNext_State;
  private boolean enableTuning = false;

  public ArbitrarySPIDF(MechanismConstants<?> config) {
    this(config, "ArbitrarySPIDF");
  }

  public ArbitrarySPIDF(MechanismConstants<?> config, String profileName) {
    name = profileName;
    if (config.pid == null) {
      // TODO: make this an exception
    }
    if (config.simpleFF == null) {
      // TODO: make this an exception
    }
    if (config.arbitraryForwardS == null) {
      // TODO: make this an exception
    }
    if (config.arbitraryReverseS == null) {
      // TODO: make this an exception
    }
    if (config.minVel == null) {
      // TODO: make this an exception
    }
    if (config.maxPosError == null) {
      // TODO: make this an exception
    }
    if (!PosVel_State.class.isAssignableFrom(config.start_State.getClass())) {
      // TODO: make this an exception
    }
    pid = config.pid;
    ff = config.simpleFF;
    arbitraryForwardS = config.arbitraryForwardS;
    arbitraryReverseS = config.arbitraryReverseS;
    minVel = config.minVel;

    maxPosError = config.maxPosError;
    lastNext_State = (PosVel_State) config.start_State;
    LogUtil.createTunablePID(config.tuningLogName + "/" + name, pid, () -> enableTuning);
    LogUtil.createTunableFF(config.tuningLogName + "/" + name, ff, () -> enableTuning);
    new TunableDouble(config.tuningLogName + "/" + name + "/minVel", minVel, () -> enableTuning);
    new TunableDouble(
        config.tuningLogName + "/" + name + "/maxPosError", maxPosError, () -> enableTuning);
  }

  public Voltage_State calculate(PosVel_State next_State, PosVel_State mechanism_State) {
    double arbitraryStatic_V;
    if (Math.abs(next_State.vel()) < minVel) {
      if (Math.abs(next_State.pos() - mechanism_State.pos()) < maxPosError) {
        arbitraryStatic_V = (arbitraryForwardS.get(mechanism_State.pos())
                + arbitraryReverseS.get(mechanism_State.pos()))
            / 2;
      } else {
        arbitraryStatic_V = next_State.pos() - mechanism_State.pos() > 0
            ? arbitraryForwardS.get(mechanism_State.pos())
            : arbitraryReverseS.get(mechanism_State.pos());
      }
    } else {
      arbitraryStatic_V = next_State.vel() > 0
          ? arbitraryForwardS.get(mechanism_State.pos())
          : arbitraryReverseS.get(mechanism_State.pos());
    }
    Voltage_State voltage = new Voltage_State(ff.calculateWithVelocities(
            lastNext_State.vel(RadiansPerSecond), next_State.vel(RadiansPerSecond))
        + arbitraryStatic_V
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
