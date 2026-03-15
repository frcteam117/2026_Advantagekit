package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO {
  public static class IntakeIOInputs implements LoggableInputs {
    public final MutAngle pivot_Pos = Radians.mutable(0);
    public final MutAngularVelocity pivot_Vel = RadiansPerSecond.mutable(0);
    public final MutAngularVelocity roller_Vel = RadiansPerSecond.mutable(0);

    @Override
    public void toLog(LogTable table) {
      table.put("Pivot_Pos", pivot_Pos.magnitude(), pivot_Pos.unit().name());
      table.put("Pivot_Vel", pivot_Vel.magnitude(), pivot_Vel.unit().name());
      table.put("Roller_Vel", roller_Vel.magnitude(), roller_Vel.unit().name());
    }

    @Override
    public void fromLog(LogTable table) {
      pivot_Pos.mut_setMagnitude(table.get("Pivot_Pos", pivot_Pos.magnitude()));
      pivot_Vel.mut_setMagnitude(table.get("Pivot_Vel", pivot_Vel.magnitude()));
      roller_Vel.mut_setMagnitude(table.get("Roller_Vel", roller_Vel.magnitude()));
    }
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPivotVoltage(Voltage voltage) {}

  /** Set the roller speed with a value from -1 to 1. */
  public default void setRollerSpeed(double speed) {}
}
