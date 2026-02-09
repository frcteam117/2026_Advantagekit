package frc.robot.util.control_functions.profiling;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.States.Angular;
import frc.robot.util.States.AngularPV_State;
import frc.robot.util.States.LinearPV_State;
import frc.robot.util.States.PosVel_State;
import frc.robot.util.States.Pos_State;
import frc.robot.util.States.VelAcc_State;
import frc.robot.util.States.Vel_State;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.logging.TunableBoolean;
import frc.robot.util.logging.TunableDouble;
import frc.robot.util.mechanisms.MechanismConstants;

public class TrapezoidProfileFunction extends ControlFunctionBase {
  private String profileName;
  private TrapezoidProfile trapezoidProfile;
  private TrapezoidProfile.State prevState;
  private double period_s;
  private boolean isAngular;
  private double maxError_Pos;
  private double min_Pos;
  private double max_Pos;
  private double max_Vel;
  private double max_Acc;
  private boolean isContinuous;
  private boolean enableTuning = false;

  public TrapezoidProfileFunction(TrapezoidProfileConfig config) {
    profileName = config.name;
    period_s = config.period_s;
    min_Pos = config.min.pos();
    max_Pos = config.max.pos();
    maxError_Pos = (max_Pos - min_Pos) / 2;
    max_Vel = ((VelAcc_State) config.constraints).vel();
    max_Acc = ((VelAcc_State) config.constraints).acc();
    isContinuous = config.enableContinuousInput;
    isAngular = Angular.class.isAssignableFrom(config.start.getClass());
    trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(max_Vel, max_Acc));
    prevState = new TrapezoidProfile.State(
        ((PosVel_State) config.start).pos(), ((PosVel_State) config.start).vel());
    String profileTuningName = config.mechanismTuningLogName + "/" + config.name;
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
        profileTuningName + "/4 isContinuous",
        isContinuous,
        () -> enableTuning,
        value -> isContinuous = value);
  }

  public TrapezoidProfileFunction(MechanismConstants config) {
    this(config, "TrapezoidProfile");
  }

  public TrapezoidProfileFunction(MechanismConstants config, String profileName) {
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
    if (config.isContinuous == null) {
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
    isContinuous = config.isContinuous;
    isAngular = Angular.class.isAssignableFrom(config.start_State.getClass());
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
        profileTuningName + "/4 isContinuous",
        isContinuous,
        () -> enableTuning,
        value -> isContinuous = value);
  }

  public PosVel_State calculate(Vel_State goal_State, PosVel_State mechanism_State) {
    prevState = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            Math.min(max_Vel, Math.abs(goal_State.vel())), max_Acc))
        .calculate(
            period_s,
            prevState,
            new TrapezoidProfile.State(
                isContinuous
                    ? (goal_State.vel() < 0) ? -Double.MAX_VALUE : Double.MAX_VALUE
                    : (goal_State.vel() < 0) ? min_Pos : max_Pos,
                goal_State.vel()));
    return isAngular
        ? new AngularPV_State(prevState.position, prevState.velocity)
        : new LinearPV_State(prevState.position, prevState.velocity);
  }

  public PosVel_State calculate(PosVel_State goal_State, PosVel_State mechanism_State) {
    prevState = trapezoidProfile.calculate(
        period_s,
        prevState,
        new TrapezoidProfile.State(
            isContinuous
                ? prevState.position
                    + MathUtil.inputModulus(
                        goal_State.pos() - prevState.position, -maxError_Pos, maxError_Pos)
                : goal_State.pos(),
            goal_State.vel()));
    return isAngular
        ? new AngularPV_State(prevState.position, prevState.velocity)
        : new LinearPV_State(prevState.position, prevState.velocity);
  }

  public PosVel_State calculate(Pos_State goal_State, PosVel_State mechanism_State) {
    prevState = trapezoidProfile.calculate(
        period_s,
        prevState,
        new TrapezoidProfile.State(
            isContinuous
                ? prevState.position
                    + MathUtil.inputModulus(
                        goal_State.pos() - prevState.position, -maxError_Pos, maxError_Pos)
                : goal_State.pos(),
            0));
    return isAngular
        ? new AngularPV_State(prevState.position, prevState.velocity)
        : new LinearPV_State(prevState.position, prevState.velocity);
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

  public static class TrapezoidProfileConfig {
    public String name = "TrapezoidProfile";
    public String mechanismTuningLogName;
    /** When true: treats min and max position as the same point on a loop. */
    public boolean enableContinuousInput = false;

    public double period_s;

    public PosVel_State start;

    public Pos_State min;
    public Pos_State max;
    public VelAcc_State constraints;
  }
}
