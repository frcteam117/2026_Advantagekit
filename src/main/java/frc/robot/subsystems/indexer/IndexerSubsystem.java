package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerConstants.*;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {
  private final SparkMax hopperSpark = new SparkMax(Hopper.CAN_ID, MotorType.kBrushless);
  private final SparkMax kickerSpark = new SparkMax(Kicker.CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder hopperEncoder = hopperSpark.getEncoder();
  private final RelativeEncoder kickerEncoder = kickerSpark.getEncoder();
  private final MutLinearVelocity hopperSurfaceVel = new MutLinearVelocity(0, 0, MetersPerSecond);
  private final MutLinearVelocity kickerSurfaceVel = new MutLinearVelocity(0, 0, MetersPerSecond);
  private boolean kickerRunningForward = false;
  public Boolean isPreloading = false;

  public IndexerSubsystem() {
    hopperSpark.configure(
        Hopper.SPARK_MAX_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kickerSpark.configure(
        Kicker.SPARK_MAX_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    hopperSurfaceVel.mut_replace(
        hopperEncoder.getVelocity()
            * 2
            * Math.PI
            * Hopper.RADIUS.in(Meters)
            / (Hopper.REDUCTION * 60),
        MetersPerSecond);
    Logger.recordOutput(Hopper.NT_KEY + "/SurfaceVel", hopperSurfaceVel);
    kickerSurfaceVel.mut_replace(
        kickerEncoder.getVelocity()
            * 2
            * Math.PI
            * Kicker.RADIUS.in(Meters)
            / (Kicker.REDUCTION * 60),
        MetersPerSecond);
    Logger.recordOutput(Kicker.NT_KEY + "/SurfaceVel", kickerSurfaceVel);
  }

  public Pose3d[] getPose3ds() {
    return new Pose3d[0];
  }

  // Hopper

  public void setHopperSpeed(double speed) {
    hopperSpark.set(speed);
    Logger.recordOutput(Hopper.NT_KEY + "/GoalSpeed", speed);
  }

  public LinearVelocity getHopperSurfaceSpeed() {
    return hopperSurfaceVel;
  }

  // Kicker

  public void setKickerSpeed(double speed) {
    kickerSpark.set(speed);
    if (speed > 0.2) {
      kickerRunningForward = true;
    }
    else {
      kickerRunningForward = false;
    }
    Logger.recordOutput(Kicker.NT_KEY + "/GoalSpeed", speed);
  }

  public LinearVelocity getKickerSurfaceSpeed() {
    return kickerSurfaceVel;
  }

  public boolean isKickerRunningForward() {
    return kickerRunningForward;
  }
}
