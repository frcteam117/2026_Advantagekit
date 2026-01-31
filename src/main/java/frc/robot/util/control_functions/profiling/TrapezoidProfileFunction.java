package frc.robot.util.control_functions.profiling;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.States.Angular;
import frc.robot.util.States.AngularPV_State;
import frc.robot.util.States.LinearPV_State;
import frc.robot.util.States.LinearP_State;
import frc.robot.util.States.LinearVA_State;
import frc.robot.util.States.PosVel_State;
import frc.robot.util.States.Pos_State;
import frc.robot.util.States.VelAcc_State;
import frc.robot.util.States.Vel_State;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.logging.TunableBoolean;
import frc.robot.util.logging.TunableDouble;

public class TrapezoidProfileFunction extends ControlFunctionBase {
  private TrapezoidProfileConfig config;
  private TrapezoidProfile trapezoidProfile;
  private TrapezoidProfile.State prevState;
  private boolean isAngular;
  public double maxError_Pos;
  private boolean enableTuning = false;

  public TrapezoidProfileFunction(TrapezoidProfileConfig config) {
    trapezoidProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(config.constraints.vel(), config.constraints.acc()));
    prevState = new TrapezoidProfile.State(config.start.pos(), config.start.vel());
    isAngular = Angular.class.isAssignableFrom(config.start.getClass());
    maxError_Pos = (config.max.pos() - config.min.pos()) / 2;
    this.config = config;
    new TunableDouble(
        "Tuning/" + config.mechanismTuningLogName + "/" + config.name + "/MinPos",
        config.min.pos(),
        () -> enableTuning,
        value -> {
          config.min = new LinearP_State(value);
          maxError_Pos = (config.max.pos() - config.min.pos()) / 2;
        });
    new TunableDouble(
        config.mechanismTuningLogName + "/" + config.name + "/MaxPos",
        config.max.pos(),
        () -> enableTuning,
        value -> {
          config.max = new LinearP_State(value);
          maxError_Pos = (config.max.pos() - config.min.pos()) / 2;
        });
    new TunableDouble(
        config.mechanismTuningLogName + "/" + config.name + "/MaxVel",
        config.constraints.vel(),
        () -> enableTuning,
        value -> {
          config.constraints = new LinearVA_State(value, config.constraints.acc());
          trapezoidProfile = new TrapezoidProfile(
              new TrapezoidProfile.Constraints(config.constraints.vel(), config.constraints.acc()));
        });
    new TunableDouble(
        config.mechanismTuningLogName + "/" + config.name + "/MaxAcc",
        config.constraints.acc(),
        () -> enableTuning,
        value -> {
          config.constraints = new LinearVA_State(config.constraints.vel(), value);
          trapezoidProfile = new TrapezoidProfile(
              new TrapezoidProfile.Constraints(config.constraints.vel(), config.constraints.acc()));
        });
    new TunableBoolean(
        config.mechanismTuningLogName + "/" + config.name + "/Continuous",
        config.enableContinuousInput,
        () -> enableTuning,
        value -> config.enableContinuousInput = value);
  }

  public PosVel_State calculate(Vel_State goal_State, PosVel_State mechanism_State) {
    prevState = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            Math.min(config.constraints.vel(), Math.abs(goal_State.vel())),
            config.constraints.acc()))
        .calculate(
            config.period_s,
            prevState,
            new TrapezoidProfile.State(
                config.enableContinuousInput
                    ? (goal_State.vel() < 0) ? -Double.MAX_VALUE : Double.MAX_VALUE
                    : (goal_State.vel() < 0) ? config.min.pos() : config.max.pos(),
                goal_State.vel()));
    return isAngular
        ? new AngularPV_State(prevState.position, prevState.velocity)
        : new LinearPV_State(prevState.position, prevState.velocity);
  }

  public PosVel_State calculate(PosVel_State goal_State, PosVel_State mechanism_State) {
    prevState = trapezoidProfile.calculate(
        config.period_s,
        prevState,
        new TrapezoidProfile.State(
            config.enableContinuousInput
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
        config.period_s,
        prevState,
        new TrapezoidProfile.State(
            config.enableContinuousInput
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
    return config.name;
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
