package frc.robot.subsystems.shooter;

import static frc.robot.Constants.robotPeriod_s;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.shooter.ShooterConstants.Flywheel;
import frc.robot.subsystems.shooter.ShooterConstants.Hood;
import frc.robot.subsystems.shooter.ShooterConstants.Turret;
import frc.robot.util.logging.LogUtil.AngularMechanismState;

public class ShooterIOSim implements ShooterIO {
  // TODO: update to use LinearSystemId.identifyVelocitySystem once built
  // Simulator
  private final FlywheelSim flywheel = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(Flywheel.gearbox, Flywheel.moi_kgm2, Flywheel.reduction),
      Flywheel.gearbox);
  private final SingleJointedArmSim hood = new SingleJointedArmSim(
      LinearSystemId.createSingleJointedArmSystem(Hood.gearbox, Hood.moi_kgm2, Hood.reduction),
      Hood.gearbox,
      Hood.reduction,
      2 * Hood.cmRadius_m,
      Hood.min_rad + Hood.cmAngle_rad,
      Hood.max_rad + Hood.cmAngle_rad,
      true,
      0 + Hood.cmAngle_rad);
  private final DCMotorSim turret = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(Turret.gearbox, Turret.moi_kgm2, Turret.reduction),
      Turret.gearbox);

  // Motion profiling
  private double flywheel_radPs = 0;
  private double flywheelLastNext_radPs = 0;
  private double hood_V = 0;
  private double hood_rad = Hood.start_rad;
  private double hood_radPs = 0;
  private double hoodLastNext_rad = 0;
  private double turret_rad = Turret.start_rad;
  private double turret_radPs = 0;
  private double turretLastNext_rad = 0;

  public ShooterIOSim() {
    reset(Hood.start_rad, Turret.start_rad);
  }

  @Override
  public void updateInputs(ShooterIOInputs ioInputs) {
    ioInputs.flywheel = new AngularMechanismState(
        0,
        flywheel.getAngularVelocityRadPerSec(),
        flywheel.getInputVoltage(),
        flywheel.getCurrentDrawAmps(),
        flywheel.getCurrentDrawAmps() * flywheel.getInputVoltage() / RoboRioSim.getVInVoltage());
    flywheel_radPs = ioInputs.flywheel.mechanism_radPs();

    ioInputs.hood = new AngularMechanismState(
        hood.getAngleRads() - Hood.cmAngle_rad,
        hood.getVelocityRadPerSec(),
        hood_V,
        hood.getCurrentDrawAmps(),
        hood.getCurrentDrawAmps() * hood_V / RoboRioSim.getVInVoltage());
    hood_rad = ioInputs.hood.mechanism_rad();
    hood_radPs = ioInputs.hood.mechanism_radPs();

    ioInputs.turret = new AngularMechanismState(
        turret.getAngularPositionRad(),
        turret.getAngularVelocityRadPerSec(),
        turret.getInputVoltage(),
        turret.getCurrentDrawAmps(),
        turret.getCurrentDrawAmps() * turret.getInputVoltage() / RoboRioSim.getVInVoltage());
    turret_rad = ioInputs.turret.mechanism_rad();
    turret_radPs = ioInputs.turret.mechanism_radPs();

    // TODO: add collision for the turret
    flywheel.update(robotPeriod_s);
    hood.update(robotPeriod_s);
    turret.update(robotPeriod_s);
  }

  @Override
  public void reset(double hood_rad, double turret_rad) {
    flywheel.setAngularVelocity(0);
    hood.setState(hood_rad + Hood.cmAngle_rad, 0);
    turret.setState(turret_rad, 0);
  }

  @Override
  public void setFlywheelVoltage(double V) {
    flywheel.setInputVoltage(V);
    flywheelLastNext_radPs = flywheel_radPs;
  }

  @Override
  public void setHoodVoltage(double V) {
    hood_V = V;
    hood.setInputVoltage(hood_V);
    hoodLastNext_rad = hood_radPs;
  }

  @Override
  public void setTurretVoltage(double V) {
    turret.setInputVoltage(V);
    turretLastNext_rad = turret_radPs;
  }

  @Override
  public void setFlywheelNextState(double next_radPs) {
    flywheel.setInputVoltage(Flywheel.simFF.calculateWithVelocities(flywheel_radPs, next_radPs)
        + Flywheel.simPID.calculate(flywheel_radPs, flywheelLastNext_radPs));
    flywheelLastNext_radPs = next_radPs;
  }

  @Override
  public void setHoodNextState(double next_rad, double next_radPs) {
    hood_V = Hood.simFF.calculateWithVelocities(hood_rad + Hood.cmAngle_rad, hood_radPs, next_radPs)
        + Hood.simPID.calculate(hood_rad, hoodLastNext_rad);
    hood.setInputVoltage(hood_V);
    hoodLastNext_rad = next_rad;
  }

  @Override
  public void setTurretNextState(double next_rad, double next_radPs) {
    turret.setInputVoltage(Turret.simFF.calculateWithVelocities(turret_radPs, next_radPs)
        + Hood.simPID.calculate(turret_rad, turretLastNext_rad));
    turretLastNext_rad = next_rad;
  }
}
