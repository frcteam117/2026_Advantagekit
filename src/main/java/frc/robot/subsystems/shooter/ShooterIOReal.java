package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.shooter.ShooterConstants.Flywheel;
import frc.robot.subsystems.shooter.ShooterConstants.Hood;
import frc.robot.subsystems.shooter.ShooterConstants.Turret;
import frc.robot.util.logging.LogUtil.AngularMechanismState;

public class ShooterIOReal implements ShooterIO {
  // Sparkmax objects
  private final SparkMax flywheel = new SparkMax(Flywheel.canId, MotorType.kBrushless);
  private final RelativeEncoder flywheelEncoder = flywheel.getEncoder();
  private final SparkMax hood = new SparkMax(Hood.canId, MotorType.kBrushless);
  private final RelativeEncoder hoodEncoder = hood.getEncoder();
  private final SparkMax turret = new SparkMax(Turret.canId, MotorType.kBrushless);
  private final RelativeEncoder turretEncoder = turret.getEncoder();

  // Motion profiling
  private double flywheel_radPs = 0;
  private double flywheelLastNext_radPs = 0;
  private double hood_rad = Hood.start_rad;
  private double hood_radPs = 0;
  private double hoodLastNext_radPs = 0;
  private double turret_rad = Turret.start_rad;
  private double turret_radPs = 0;
  private double turretLastNext_radPs = 0;

  public ShooterIOReal() {
    flywheel.configure(
        Flywheel.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hood.configure(
        Hood.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turret.configure(
        Turret.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    reset(Hood.start_rad, Turret.start_rad);
  }

  @Override
  public void updateInputs(ShooterIOInputs ioInputs) {
    ioInputs.flywheel = new AngularMechanismState(
        flywheelEncoder.getPosition(),
        flywheelEncoder.getVelocity(),
        flywheel.getBusVoltage() * flywheel.getAppliedOutput(),
        flywheel.getOutputCurrent(),
        flywheel.getOutputCurrent() * flywheel.getAppliedOutput());
    flywheel_radPs = ioInputs.flywheel.mechanism_radPs();

    ioInputs.hood = new AngularMechanismState(
        hoodEncoder.getPosition(),
        hoodEncoder.getVelocity(),
        hood.getBusVoltage() * hood.getAppliedOutput(),
        hood.getOutputCurrent(),
        hood.getOutputCurrent() * hood.getAppliedOutput());
    hood_rad = ioInputs.hood.mechanism_rad();
    hood_radPs = ioInputs.hood.mechanism_radPs();

    ioInputs.turret = new AngularMechanismState(
        turretEncoder.getPosition(),
        turretEncoder.getVelocity(),
        turret.getBusVoltage() * turret.getAppliedOutput(),
        turret.getOutputCurrent(),
        turret.getOutputCurrent() * turret.getAppliedOutput());
    turret_rad = ioInputs.turret.mechanism_rad();
    turret_radPs = ioInputs.turret.mechanism_radPs();
  }

  @Override
  public void reset(double hood_rad, double turret_rad) {
    flywheelEncoder.setPosition(0);
    hoodEncoder.setPosition(hood_rad);
    turretEncoder.setPosition(turret_rad);
  }

  @Override
  public void setFlywheelVoltage(double V) {
    flywheel.setVoltage(V);
    flywheelLastNext_radPs = flywheel_radPs;
  }

  @Override
  public void setHoodVoltage(double V) {
    hood.setVoltage(V);
    hoodLastNext_radPs = hood_radPs;
  }

  @Override
  public void setTurretVoltage(double V) {
    turret.setVoltage(V);
    turretLastNext_radPs = turret_radPs;
  }

  @Override
  public void setFlywheelNextState(double next_radPs) {
    flywheel.setVoltage(Flywheel.realFF.calculateWithVelocities(flywheel_radPs, next_radPs)
        + Flywheel.realPID.calculate(flywheel_radPs, flywheelLastNext_radPs));
    flywheelLastNext_radPs = next_radPs;
  }

  @Override
  public void setHoodNextState(double next_rad, double next_radPs) {
    hood.setVoltage(Hood.realFF.calculateWithVelocities(hood_rad, hood_radPs, next_radPs)
        + Hood.realPID.calculate(hood_rad, hoodLastNext_radPs));
    hoodLastNext_radPs = next_radPs;
  }

  @Override
  public void setTurretNextState(double next_rad, double next_radPs) {
    turret.setVoltage(Turret.realFF.calculateWithVelocities(turret_radPs, next_radPs)
        + Hood.realPID.calculate(turret_rad, turretLastNext_radPs));
    turretLastNext_radPs = next_radPs;
  }
}
