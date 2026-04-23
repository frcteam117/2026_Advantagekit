package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterIOReal implements ShooterIO {
  private final SparkMax hood_Spark = new SparkMax(Hood.CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder hood_Encoder = hood_Spark.getEncoder();

  private final SparkMax rioFlywheel_Leader =
      new SparkMax(RIO_Flywheel.LEADER_CAN_ID, MotorType.kBrushless);
  private final SparkMax rioFlywheel_Follower =
      new SparkMax(RIO_Flywheel.FOLLOWER_CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder rioFlywheel_Encoder = rioFlywheel_Leader.getEncoder();

  private final SparkMax pdhFlywheel_Leader =
      new SparkMax(PDH_Flywheel.LEADER_CAN_ID, MotorType.kBrushless);
  private final SparkMax pdhFlywheel_Follower =
      new SparkMax(PDH_Flywheel.FOLLOWER_CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder pdhFlywheel_Encoder = pdhFlywheel_Leader.getEncoder();

  public ShooterIOReal() {
    hood_Spark.configure(
        Hood.SPARK_MAX_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rioFlywheel_Leader.configure(
        RIO_Flywheel.SPARK_MAX_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rioFlywheel_Follower.configure(
        RIO_Flywheel.SPARK_MAX_CONFIG.follow(rioFlywheel_Leader, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    pdhFlywheel_Leader.configure(
        PDH_Flywheel.SPARK_MAX_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    pdhFlywheel_Follower.configure(
        PDH_Flywheel.SPARK_MAX_CONFIG.follow(pdhFlywheel_Leader, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.hood_Pos.mut_replace(hood_Encoder.getPosition() * 2 * Math.PI / Hood.REDUCTION, Radians);
    inputs.hood_Vel.mut_replace(
        hood_Encoder.getVelocity() * 2 * Math.PI / (Hood.REDUCTION * 60), RadiansPerSecond);
    inputs.rioFlywheel_Vel.mut_replace(
        rioFlywheel_Encoder.getVelocity() * 2 * Math.PI / (RIO_Flywheel.REDUCTION * 60),
        RadiansPerSecond);
    inputs.pdhFlywheel_Vel.mut_replace(
        pdhFlywheel_Encoder.getVelocity() * 2 * Math.PI / (PDH_Flywheel.REDUCTION * 60),
        RadiansPerSecond);
    inputs.rioFlywheel_AppliedOutput = rioFlywheel_Leader.getAppliedOutput();
    inputs.pdhFlywheel_AppliedOutput = pdhFlywheel_Leader.getAppliedOutput();
  }

  @Override
  public void setHoodVoltage(Voltage voltage) {
    hood_Spark.setVoltage(voltage);
  }

  @Override
  public void setRIOFlywheelVoltage(Voltage voltage) {
    rioFlywheel_Leader.setVoltage(voltage);
  }

  @Override
  public void setPDHFlywheelVoltage(Voltage voltage) {
    pdhFlywheel_Leader.setVoltage(voltage);
  }
}
