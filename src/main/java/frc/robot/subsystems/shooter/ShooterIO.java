package frc.robot.subsystems.shooter;

import frc.robot.util.logging.LogUtil.AngularMechanismState;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public AngularMechanismState flywheel;
    public AngularMechanismState hood;
    public AngularMechanismState turret;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void reset(double hood_rad, double turret_rad) {}

  public default void setFlywheelVoltage(double V) {}

  public default void setHoodVoltage(double V) {}

  public default void setTurretVoltage(double V) {}

  public default void setFlywheelNextState(double next_radPs) {}

  public default void setHoodNextState(double next_rad, double next_radPs) {}

  public default void setTurretNextState(double next_rad, double next_radPs) {}
}
