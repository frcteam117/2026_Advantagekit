package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.intake.IntakeConstants.*;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.util.logging.LogUtil;
import frc.robot.util.logging.TunableBoolean;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  // Pivot
  private final TrapezoidProfile.Constraints pivot_Constraints;
  private final TrapezoidProfile pivot_Profile;
  private final PIDController pivot_PID;
  private final SimpleMotorFeedforward pivot_FF;
  private final InterpolatingDoubleTreeMap pivot_ArbitraryFF = new InterpolatingDoubleTreeMap();

  private TrapezoidProfile.State pivot_PrevNextState;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;

    pivot_Constraints = new TrapezoidProfile.Constraints(5, 8);
    pivot_Profile = new TrapezoidProfile(pivot_Constraints);

    if (RobotBase.isReal()) {
      pivot_PID = new PIDController(20, 0, 0, RobotConstants.CODE_PERIOD_s);
      pivot_FF = new SimpleMotorFeedforward(0.2, 1.3, 0.05, RobotConstants.CODE_PERIOD_s);
      pivot_ArbitraryFF.put(0.1 + 1.41, 0.0);
      pivot_ArbitraryFF.put(-0.1 + 1.41, 0.0);
      pivot_ArbitraryFF.put(-0.4 + 1.41, 0.0);
      pivot_ArbitraryFF.put(-0.7 + 1.41, 0.01);
      pivot_ArbitraryFF.put(-1.0 + 1.41, 0.045);
      pivot_ArbitraryFF.put(-1.3 + 1.41, 0.075);
      pivot_ArbitraryFF.put(-1.6 + 1.41, 0.09);
    } else {
      pivot_PID = new PIDController(0, 0, 0, RobotConstants.CODE_PERIOD_s);
      pivot_FF = new SimpleMotorFeedforward(0, 0, 0, RobotConstants.CODE_PERIOD_s);
      pivot_ArbitraryFF.put(-1000000.0, 0.0);
      pivot_ArbitraryFF.put(1000.0, 0.0);
    }

    final String pivot_tuningNTKey = RobotConstants.TUNING_PREFIX + Pivot.NT_KEY;
    final BooleanSupplier pivot_Tunable =
        new TunableBoolean(pivot_tuningNTKey + "/.tunable", false);
    LogUtil.createTunablePID(pivot_tuningNTKey + "/PID", pivot_PID, pivot_Tunable);
    LogUtil.createTunableFF(pivot_tuningNTKey + "/SimpleFF", pivot_FF, pivot_Tunable);
    LogUtil.createTunableLerpTable(
        pivot_tuningNTKey + "/ArbirtaryFF",
        pivot_ArbitraryFF,
        pivot_Tunable,
        0.1 + 1.41,
        -0.1 + 1.41,
        -0.4 + 1.41,
        -0.7 + 1.41,
        -1.0 + 1.41,
        -1.3 + 1.41,
        -1.6 + 1.41);

    // final String roller_tuningNTKey = RobotConstants.TUNING_PREFIX + Roller.NT_KEY;
    // final BooleanSupplier roller_Tunable = new TunableBoolean(roller_tuningNTKey + "/.tunable",
    // false);

    periodic();
    setPivotVoltage(Volts.of(0.0));
    setRollerSpeed(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(IntakeConstants.NT_KEY, inputs);
  }

  public Pose3d[] getPose3ds() {
    return getIntakePose(getPivotPos().in(Radians));
  }

  public Angle getPivotPos() {
    return inputs.pivot.position;
  }

  public AngularVelocity getPivotVel() {
    return inputs.pivot.velocity;
  }

  public AngularVelocity getRollerVel() {
    return inputs.roller.velocity;
  }

  // Pivot

  public void setPivotVoltage(Voltage voltage) {
    io.setPivotVoltage(voltage);
    Logger.recordOutput(Pivot.NT_KEY + "/1_GoalPos", Double.NaN, Radians);
    Logger.recordOutput(Pivot.NT_KEY + "/1_GoalVel", Double.NaN, RadiansPerSecond);
    Logger.recordOutput(Pivot.NT_KEY + "/2_NextPos", Double.NaN, Radians);
    Logger.recordOutput(Pivot.NT_KEY + "/2_NextVel", Double.NaN, RadiansPerSecond);
    Logger.recordOutput(Pivot.NT_KEY + "/3_OutputVoltage", voltage);
    pivot_PrevNextState = new TrapezoidProfile.State(
        inputs.pivot.position.in(Radians), inputs.pivot.velocity.in(RadiansPerSecond));
  }

  public void setPivotGoalPos(Angle goalPos) {
    TrapezoidProfile.State nextState = pivot_Profile.calculate(
        RobotConstants.CODE_PERIOD_s,
        pivot_PrevNextState,
        new TrapezoidProfile.State(
            Math.max(
                Pivot.MIN_POS.in(Radians),
                Math.min(Pivot.MAX_POS.in(Radians), goalPos.in(Radians))),
            0));
    Voltage voltage = Volts.of(pivot_ArbitraryFF.get(inputs.pivot.position.in(Radians))
        + pivot_FF.calculateWithVelocities(pivot_PrevNextState.velocity, nextState.velocity)
        + pivot_PID.calculate(inputs.pivot.position.in(Radians), pivot_PrevNextState.position));
    io.setPivotVoltage(voltage);
    Logger.recordOutput(Pivot.NT_KEY + "/1_GoalPos", goalPos.in(Radians), Radians);
    Logger.recordOutput(Pivot.NT_KEY + "/1_GoalVel", Double.NaN, RadiansPerSecond);
    Logger.recordOutput(Pivot.NT_KEY + "/2_NextPos", nextState.position, Radians);
    Logger.recordOutput(Pivot.NT_KEY + "/2_NextVel", nextState.velocity, RadiansPerSecond);
    Logger.recordOutput(Pivot.NT_KEY + "/3_OutputVoltage", voltage);
    pivot_PrevNextState = nextState;
  }

  public void setPivotGoalVel(AngularVelocity goalVel) {
    TrapezoidProfile.State nextState = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            Math.max(
                Math.abs(pivot_PrevNextState.velocity)
                    - pivot_Constraints.maxAcceleration * RobotConstants.CODE_PERIOD_s,
                Math.min(pivot_Constraints.maxVelocity, Math.abs(goalVel.in(RadiansPerSecond)))),
            pivot_Constraints.maxAcceleration))
        .calculate(
            RobotConstants.CODE_PERIOD_s,
            pivot_PrevNextState,
            new TrapezoidProfile.State(
                (goalVel.magnitude() < 0) ? Pivot.MIN_POS.in(Radians) : Pivot.MAX_POS.in(Radians),
                0));
    Voltage voltage = Volts.of(pivot_ArbitraryFF.get(inputs.pivot.position.in(Radians))
        + pivot_FF.calculateWithVelocities(pivot_PrevNextState.velocity, nextState.velocity)
        + pivot_PID.calculate(inputs.pivot.position.in(Radians), pivot_PrevNextState.position));
    io.setPivotVoltage(voltage);
    Logger.recordOutput(Pivot.NT_KEY + "/1_GoalPos", Double.NaN, Radians);
    Logger.recordOutput(
        Pivot.NT_KEY + "/1_GoalVel", goalVel.in(RadiansPerSecond), RadiansPerSecond);
    Logger.recordOutput(Pivot.NT_KEY + "/2_NextPos", nextState.position, Radians);
    Logger.recordOutput(Pivot.NT_KEY + "/2_NextVel", nextState.velocity, RadiansPerSecond);
    Logger.recordOutput(Pivot.NT_KEY + "/3_OutputVoltage", voltage);
    pivot_PrevNextState = nextState;
  }

  // Roller
  public void setRollerSpeed(double speed) {
    io.setRollerSpeed(speed);
    Logger.recordOutput(Roller.NT_KEY + "/3_OutputSpeed", speed);
  }

  private Pose3d[] getIntakePose(double rad) {
    // math based on https://www.desmos.com/calculator/8e3rmxmen2
    double g = Math.sqrt(0.081730601 - 0.0338354902642 * Math.cos(rad));
    double ifVar = Math.PI < rad % (2 * Math.PI) ? 1 : -1;
    double theta_a = rad - 0.564035611052;
    double theta_b = 2.55195984092
        - Math.acos((-0.0732526549 - (g * g)) / (-0.558793904763 * g))
        + ifVar * Math.acos((0.0743979216 - (g * g)) / (-0.121100614367 * g));
    double theta_e = 3.8131545094
        + theta_a
        + Math.acos((0.0732526549 - (g * g)) / (-0.138708357355 * g))
        - ifVar * Math.acos((-0.0743979216 - (g * g)) / (-0.558799646743 * g));
    // these are of b_1 - a_1 on the graph
    double x = 0.06006
        + (0.253999673228 * Math.cos(0.462274273714 + theta_b))
        - (0.217012684652 * Math.cos(0.471540446498 + theta_a));
    double y = -0.00769
        + (0.253999673228 * Math.sin(0.462274273714 + theta_b))
        - (0.217012684652 * Math.sin(0.471540446498 + theta_a));
    double h = Math.sqrt(x * x + y * y);
    double atanOfh = Math.atan2(y, x);
    double theta_c =
        atanOfh + 1.22843881587 - Math.acos((-0.0107190074 - h * h) / (-0.510740385715 * h));
    double theta_d =
        atanOfh + 4.7931822977 + Math.acos((0.0107190074 - h * h) / (-0.466882974631 * h));

    Pose3d[] poses = new Pose3d[5];
    // the thetas are negative because pitch rotates down from z to x and the thetas are the angle
    // above the normal position
    poses[0] = new Pose3d(0.16543, 0, 0.18708, new Rotation3d(0, -theta_a, 0));
    poses[1] = new Pose3d(0.22549, 0, 0.17939, new Rotation3d(0, -theta_b, 0));
    poses[2] = new Pose3d(
        new Translation3d(0.19333, 0, 0.09858)
            .rotateBy(poses[0].getRotation())
            .plus(poses[0].getTranslation()),
        new Rotation3d(0, -theta_c, 0));
    poses[3] = new Pose3d(
        new Translation3d(0.22734, 0, 0.11328)
            .rotateBy(poses[1].getRotation())
            .plus(poses[1].getTranslation()),
        new Rotation3d(0, -theta_d, 0));
    poses[4] = new Pose3d(
        new Translation3d(0.25318, 0, 0.11817)
            .rotateBy(poses[0].getRotation())
            .plus(poses[0].getTranslation()),
        new Rotation3d(0, -theta_e, 0));

    return poses;
  }
  public void setBrakeMode(boolean enabled) {
    
  }
}
