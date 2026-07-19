package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.shooter.ShooterConstants.*;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.util.logging.LogUtil;
import frc.robot.util.logging.TunableBoolean;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputs inputs = new ShooterIOInputs();

  // Hood
  private final TrapezoidProfile.Constraints hood_Constraints;
  private final TrapezoidProfile hood_Profile;
  private final PIDController hood_PID;
  private final SimpleMotorFeedforward hood_FF;

  // Rio Flywheel
  private final SlewRateLimiter rioFlywheel_AccLimiter;
  private final PIDController rioFlywheel_PID;
  private final SimpleMotorFeedforward rioFlywheel_FF;

  // PDH Flywheel
  private final SlewRateLimiter pdhFlywheel_AccLimiter;
  private final PIDController pdhFlywheel_PID;
  private final SimpleMotorFeedforward pdhFlywheel_FF;

  private TrapezoidProfile.State hood_PrevNextState;
  private double rioFlywheel_PrevNextVel_radPs;
  private double pdhFlywheel_PrevNextVel_radPs;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;

    hood_Constraints = new TrapezoidProfile.Constraints(1.8, 16);
    hood_Profile = new TrapezoidProfile(hood_Constraints);
    rioFlywheel_AccLimiter = new SlewRateLimiter(300);
    pdhFlywheel_AccLimiter = new SlewRateLimiter(300);

    if (RobotBase.isReal()) {
      // Hood
      hood_PID = new PIDController(70, 0, 0, RobotConstants.CODE_PERIOD_s);
      hood_FF = new SimpleMotorFeedforward(0.1, 6.7, .25, RobotConstants.CODE_PERIOD_s);
      // RIO Flywheel
      rioFlywheel_PID = new PIDController(0.006, 0, 0, RobotConstants.CODE_PERIOD_s);
      rioFlywheel_FF =
          new SimpleMotorFeedforward(0.23, 0.0183, 0.0013, RobotConstants.CODE_PERIOD_s);
      // PDH Flywheel
      pdhFlywheel_PID = new PIDController(0.006, 0, 0, RobotConstants.CODE_PERIOD_s);
      pdhFlywheel_FF =
          new SimpleMotorFeedforward(0.132, 0.01705, 0.0013, RobotConstants.CODE_PERIOD_s);
    } else {
      // Hood
      hood_PID = new PIDController(0, 0, 0, RobotConstants.CODE_PERIOD_s);
      hood_FF = new SimpleMotorFeedforward(0, 0, 0, RobotConstants.CODE_PERIOD_s);
      // RIO Flywheel
      rioFlywheel_PID = new PIDController(0, 0, 0, RobotConstants.CODE_PERIOD_s);
      rioFlywheel_FF = new SimpleMotorFeedforward(0, 0, 0, RobotConstants.CODE_PERIOD_s);
      // PDH Flywheel
      pdhFlywheel_PID = new PIDController(0, 0, 0, RobotConstants.CODE_PERIOD_s);
      pdhFlywheel_FF = new SimpleMotorFeedforward(0, 0, 0, RobotConstants.CODE_PERIOD_s);
    }

    final String hood_tuningNTKey = RobotConstants.TUNING_PREFIX + Hood.NT_KEY;
    final BooleanSupplier hood_Tunable = new TunableBoolean(hood_tuningNTKey + "/.tunable", false);
    LogUtil.createTunablePID(hood_tuningNTKey + "/PID", hood_PID, hood_Tunable);
    LogUtil.createTunableFF(hood_tuningNTKey + "/SimpleFF", hood_FF, hood_Tunable);

    final String rioFlywheel_tuningNTKey = RobotConstants.TUNING_PREFIX + RIO_Flywheel.NT_KEY;
    final BooleanSupplier rioFlywheel_Tunable =
        new TunableBoolean(rioFlywheel_tuningNTKey + "/.tunable", false);
    LogUtil.createTunablePID(
        rioFlywheel_tuningNTKey + "/PID", rioFlywheel_PID, rioFlywheel_Tunable);
    LogUtil.createTunableFF(
        rioFlywheel_tuningNTKey + "/SimpleFF", rioFlywheel_FF, rioFlywheel_Tunable);

    final String pdhFlywheel_tuningNTKey = RobotConstants.TUNING_PREFIX + PDH_Flywheel.NT_KEY;
    final BooleanSupplier pdhFlywheel_Tunable =
        new TunableBoolean(pdhFlywheel_tuningNTKey + "/.tunable", false);
    LogUtil.createTunablePID(
        pdhFlywheel_tuningNTKey + "/PID", pdhFlywheel_PID, pdhFlywheel_Tunable);
    LogUtil.createTunableFF(
        pdhFlywheel_tuningNTKey + "/SimpleFF", pdhFlywheel_FF, pdhFlywheel_Tunable);

    periodic();
    setHoodVoltage(Volts.zero());
    setRIOFlywheelVoltage(Volts.zero());
    setPDHFlywheelVoltage(Volts.zero());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(ShooterConstants.LOG_NAME, inputs);
  }

  public Pose3d getPose3d() {
    return new Pose3d(-0.24286, 0, 0.58996, new Rotation3d(0, -getHoodPos().in(Radians), 0));
  }

  public Angle getHoodPos() {
    return inputs.hood_Pos;
  }

  public AngularVelocity getHoodVel() {
    return inputs.hood_Vel;
  }

  public double getRevPrecentage() {
    return inputs.rioFlywheel_Vel.in(RadiansPerSecond) / rioFlywheel_PID.getSetpoint() * 100;
  }

  public AngularVelocity getRIOFlywheelVel() {
    return inputs.rioFlywheel_Vel;
  }

  public AngularVelocity getPDHFlywheelVel() {
    return inputs.pdhFlywheel_Vel;
  }

  public double getPDHFlywheelAppliedOutput() {
    return inputs.pdhFlywheel_AppliedOutput;
  }

  public double getRIOFlywheelAppliedOutput() {
    return inputs.rioFlywheel_AppliedOutput;
  }

  // Hood

  public void setHoodVoltage(Voltage voltage) {
    io.setHoodVoltage(voltage);
    Logger.recordOutput(Hood.NT_KEY + "/1_GoalPos", Double.NaN, Radians);
    Logger.recordOutput(Hood.NT_KEY + "/1_GoalVel", Double.NaN, RadiansPerSecond);
    Logger.recordOutput(Hood.NT_KEY + "/2_NextPos", Double.NaN, Radians);
    Logger.recordOutput(Hood.NT_KEY + "/2_NextVel", Double.NaN, RadiansPerSecond);
    Logger.recordOutput(Hood.NT_KEY + "/3_OutputVoltage", voltage);
    hood_PrevNextState = new TrapezoidProfile.State(
        inputs.hood_Pos.in(Radians), inputs.hood_Vel.in(RadiansPerSecond));
  }

  public void setHoodGoalPos(Angle goalPos) {
    TrapezoidProfile.State nextState = hood_Profile.calculate(
        RobotConstants.CODE_PERIOD_s,
        hood_PrevNextState,
        new TrapezoidProfile.State(
            Math.max(
                Hood.MIN_POS.in(Radians), Math.min(Hood.MAX_POS.in(Radians), goalPos.in(Radians))),
            0));
    Voltage voltage =
        Volts.of(hood_FF.calculateWithVelocities(hood_PrevNextState.velocity, nextState.velocity)
            + hood_PID.calculate(inputs.hood_Pos.in(Radians), hood_PrevNextState.position));
    io.setHoodVoltage(voltage);
    Logger.recordOutput(Hood.NT_KEY + "/1_GoalPos", goalPos.in(Radians), Radians);
    Logger.recordOutput(Hood.NT_KEY + "/1_GoalVel", Double.NaN, RadiansPerSecond);
    Logger.recordOutput(Hood.NT_KEY + "/2_NextPos", nextState.position, Radians);
    Logger.recordOutput(Hood.NT_KEY + "/2_NextVel", nextState.velocity, RadiansPerSecond);
    Logger.recordOutput(Hood.NT_KEY + "/3_OutputVoltage", voltage);
    hood_PrevNextState = nextState;
  }

  public void setHoodGoalVel(AngularVelocity goalVel) {
    TrapezoidProfile.State nextState = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            Math.max(
                Math.abs(hood_PrevNextState.velocity)
                    - hood_Constraints.maxAcceleration * RobotConstants.CODE_PERIOD_s,
                Math.min(hood_Constraints.maxVelocity, Math.abs(goalVel.in(RadiansPerSecond)))),
            hood_Constraints.maxAcceleration))
        .calculate(
            RobotConstants.CODE_PERIOD_s,
            hood_PrevNextState,
            new TrapezoidProfile.State(
                (goalVel.magnitude() < 0) ? Hood.MIN_POS.in(Radians) : Hood.MAX_POS.in(Radians),
                0));
    Voltage voltage =
        Volts.of(hood_FF.calculateWithVelocities(hood_PrevNextState.velocity, nextState.velocity)
            + hood_PID.calculate(inputs.hood_Pos.in(Radians), hood_PrevNextState.position));
    io.setHoodVoltage(voltage);
    Logger.recordOutput(Hood.NT_KEY + "/1_GoalPos", Double.NaN, Radians);
    Logger.recordOutput(Hood.NT_KEY + "/1_GoalVel", goalVel.in(RadiansPerSecond), RadiansPerSecond);
    Logger.recordOutput(Hood.NT_KEY + "/2_NextPos", nextState.position, Radians);
    Logger.recordOutput(Hood.NT_KEY + "/2_NextVel", nextState.velocity, RadiansPerSecond);
    Logger.recordOutput(Hood.NT_KEY + "/3_OutputVoltage", voltage);
    hood_PrevNextState = nextState;
  }

  // RIO Flywheel

  public void setRIOFlywheelVoltage(Voltage voltage) {
    io.setRIOFlywheelVoltage(voltage);
    Logger.recordOutput(RIO_Flywheel.NT_KEY + "/1_GoalVel", Double.NaN, RadiansPerSecond);
    Logger.recordOutput(RIO_Flywheel.NT_KEY + "/2_NextVel", Double.NaN, RadiansPerSecond);
    Logger.recordOutput(RIO_Flywheel.NT_KEY + "/3_OutputVoltage", voltage);
    rioFlywheel_PrevNextVel_radPs = inputs.rioFlywheel_Vel.in(RadiansPerSecond);
  }

  public void setRIOFlywheelGoalVel(AngularVelocity goalVel) {
    double nextVel_radPs = rioFlywheel_AccLimiter.calculate(goalVel.in(RadiansPerSecond));
    Voltage voltage = Volts.of(
        rioFlywheel_FF.calculateWithVelocities(rioFlywheel_PrevNextVel_radPs, nextVel_radPs)
            + rioFlywheel_PID.calculate(
                inputs.rioFlywheel_Vel.in(RadiansPerSecond), rioFlywheel_PrevNextVel_radPs));
    io.setRIOFlywheelVoltage(voltage);
    Logger.recordOutput(
        RIO_Flywheel.NT_KEY + "/1_GoalVel", goalVel.in(RadiansPerSecond), RadiansPerSecond);
    Logger.recordOutput(RIO_Flywheel.NT_KEY + "/2_NextVel", nextVel_radPs, RadiansPerSecond);
    Logger.recordOutput(RIO_Flywheel.NT_KEY + "/3_OutputVoltage", voltage);
    rioFlywheel_PrevNextVel_radPs = nextVel_radPs;
  }

  // PDH Flywheel

  public void setPDHFlywheelVoltage(Voltage voltage) {
    io.setPDHFlywheelVoltage(voltage);
    Logger.recordOutput(PDH_Flywheel.NT_KEY + "/1_GoalVel", Double.NaN, RadiansPerSecond);
    Logger.recordOutput(PDH_Flywheel.NT_KEY + "/2_NextVel", Double.NaN, RadiansPerSecond);
    Logger.recordOutput(PDH_Flywheel.NT_KEY + "/3_OutputVoltage", voltage);
    pdhFlywheel_PrevNextVel_radPs = inputs.pdhFlywheel_Vel.in(RadiansPerSecond);
  }

  public void setPDHFlywheelGoalVel(AngularVelocity goalVel) {
    double nextVel_radPs = pdhFlywheel_AccLimiter.calculate(goalVel.in(RadiansPerSecond));
    Voltage voltage = Volts.of(
        pdhFlywheel_FF.calculateWithVelocities(pdhFlywheel_PrevNextVel_radPs, nextVel_radPs)
            + pdhFlywheel_PID.calculate(
                inputs.pdhFlywheel_Vel.in(RadiansPerSecond), pdhFlywheel_PrevNextVel_radPs));
    io.setPDHFlywheelVoltage(voltage);
    Logger.recordOutput(
        PDH_Flywheel.NT_KEY + "/1_GoalVel", goalVel.in(RadiansPerSecond), RadiansPerSecond);
    Logger.recordOutput(PDH_Flywheel.NT_KEY + "/2_NextVel", nextVel_radPs, RadiansPerSecond);
    Logger.recordOutput(PDH_Flywheel.NT_KEY + "/3_OutputVoltage", voltage);
    pdhFlywheel_PrevNextVel_radPs = nextVel_radPs;
  }
}
