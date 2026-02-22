package frc.robot.util.control_functions.profiling;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.logging.TunableBoolean;
import frc.robot.util.logging.TunableDouble;
import frc.robot.util.mechanisms.MechanismConstants;
import frc.robot.util.states.StateValue;
import frc.robot.util.states.bases.PosVel_State;
import frc.robot.util.states.bases.Pos_State;
import frc.robot.util.states.bases.VelAcc_State;
import frc.robot.util.states.bases.Vel_State;

public class TrapezoidProfileFunction extends ControlFunctionBase {
  private final String profileName;
  private TrapezoidProfile trapezoidProfile;
  private TrapezoidProfile.State prevState;
  private final double period_s;
  private double maxError_Pos;
  private double min_Pos;
  private double max_Pos;
  private double max_Vel;
  private double max_Acc;
  private boolean isLoop;
  private boolean enableTuning = false;

  public TrapezoidProfileFunction(MechanismConstants<?> config) {
    this(config, "TrapezoidProfile");
  }

  public TrapezoidProfileFunction(MechanismConstants<?> config, String profileName) {
    if (config.start_State == null) {
      // TODO: add an exeption
    }
    if (config.limits_State == null) {
      // TODO: add an exeption
    }
    if (config.min_Pos == null) {
      // TODO: add an exeption
    }
    if (config.max_Pos == null) {
      // TODO: add an exeption
    }
    if (config.isLoop == null) {
      // TODO: add an exeption
    }
    if (config.codePeriod_s == null) {
      // TODO: add an exeption
    }
    if (!VelAcc_State.class.isAssignableFrom(config.limits_State.getClass())) {
      // TODO: add an exeption
    }
    if (!PosVel_State.class.isAssignableFrom(config.start_State.getClass())) {
      // TODO: add an exeption
    }
    this.profileName = profileName;
    period_s = config.codePeriod_s;
    min_Pos = config.min_Pos.pos();
    max_Pos = config.max_Pos.pos();
    maxError_Pos = (max_Pos - min_Pos) / 2;
    max_Vel = ((VelAcc_State) config.limits_State).vel();
    max_Acc = ((VelAcc_State) config.limits_State).acc();
    isLoop = config.isLoop;
    trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(max_Vel, max_Acc));
    prevState = new TrapezoidProfile.State(
        ((PosVel_State) config.start_State).pos(), ((PosVel_State) config.start_State).vel());
    String profileTuningName = config.tuningLogName + "/" + profileName;
    new TunableDouble(profileTuningName + "/0 Min_Pos", min_Pos, () -> enableTuning, value -> {
      min_Pos = value;
      maxError_Pos = (max_Pos - min_Pos) / 2;
    });
    new TunableDouble(profileTuningName + "/1 Max_Pos", max_Pos, () -> enableTuning, value -> {
      max_Pos = value;
      maxError_Pos = (max_Pos - min_Pos) / 2;
    });
    new TunableDouble(profileTuningName + "/2 Max_Vel", max_Vel, () -> enableTuning, value -> {
      max_Vel = value;
      trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(max_Vel, max_Acc));
    });
    new TunableDouble(profileTuningName + "/3 Max_Acc", max_Acc, () -> enableTuning, value -> {
      max_Acc = value;
      trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(max_Vel, max_Acc));
    });
    new TunableBoolean(
        profileTuningName + "/4 isLoop", isLoop, () -> enableTuning, value -> isLoop = value);
  }

  public PosVel_State calculate(Vel_State goal_State, PosVel_State mechanism_State) {
    prevState = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            Math.max(
                prevState.velocity - max_Acc * period_s,
                Math.min(max_Vel, Math.abs(goal_State.vel()))),
            max_Acc))
        .calculate(
            period_s,
            prevState,
            new TrapezoidProfile.State(
                isLoop
                    ? (goal_State.vel() < 0) ? -Double.MAX_VALUE : Double.MAX_VALUE
                    : (goal_State.vel() < 0) ? min_Pos : max_Pos,
                goal_State.vel()));
    return PosVel_State.create(
        new StateValue(prevState.position, mechanism_State.posValue().getUnit()),
        new StateValue(prevState.velocity, mechanism_State.velValue().getUnit()));
  }

  public PosVel_State calculate(PosVel_State goal_State, PosVel_State mechanism_State) {
    prevState = trapezoidProfile.calculate(
        period_s,
        prevState,
        new TrapezoidProfile.State(
            isLoop
                ? prevState.position
                    + MathUtil.inputModulus(
                        goal_State.pos() - prevState.position, -maxError_Pos, maxError_Pos)
                : goal_State.pos(),
            goal_State.vel()));
    return PosVel_State.create(
        new StateValue(prevState.position, mechanism_State.posValue().getUnit()),
        new StateValue(prevState.velocity, mechanism_State.velValue().getUnit()));
  }

  public PosVel_State calculate(Pos_State goal_State, PosVel_State mechanism_State) {
    prevState = trapezoidProfile.calculate(
        period_s,
        prevState,
        new TrapezoidProfile.State(
            isLoop
                ? prevState.position
                    + MathUtil.inputModulus(
                        goal_State.pos() - prevState.position, -maxError_Pos, maxError_Pos)
                : goal_State.pos(),
            0));
    return PosVel_State.create(
        new StateValue(prevState.position, mechanism_State.posValue().getUnit()),
        new StateValue(prevState.velocity, mechanism_State.velValue().getUnit()));
  }

  public void updateState(PosVel_State mechanism_State) {
    resetState(mechanism_State);
  }

  public void resetState(PosVel_State mechanism_State) {
    prevState.position = mechanism_State.pos();
    prevState.velocity = mechanism_State.vel();
  }

  public void resetState(Pos_State mechanism_State) {
    prevState.position = mechanism_State.pos();
  }

  public void resetState(Vel_State mechanism_State) {
    prevState.velocity = mechanism_State.vel();
  }

  @Override
  public String getControlFunctionName() {
    return profileName;
  }

  @Override
  public void setTunable(boolean isTunable) {
    enableTuning = isTunable;
  }
}
