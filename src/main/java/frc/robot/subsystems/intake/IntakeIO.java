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
    public final IntakeIOInputs.Roller roller = new IntakeIOInputs.Roller();
    public final IntakeIOInputs.Pivot pivot = new IntakeIOInputs.Pivot();

    public class Roller {
      public final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);
    }

    public class Pivot {
      public final MutAngle position = Radians.mutable(0);
      public final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);
    }
    // public final MutAngle pivot_Pos = Radians.mutable(0);
    // public final MutAngularVelocity pivot_Vel = RadiansPerSecond.mutable(0);
    // public final MutAngularVelocity roller_Vel = RadiansPerSecond.mutable(0);

    @Override
    public void toLog(LogTable table) {
      table.put("Pivot_Pos", pivot.position.in(Radians), Radians.name());
      table.put("Pivot_Vel", pivot.velocity.in(RadiansPerSecond), RadiansPerSecond.name());
      table.put("Roller_Vel", roller.velocity.in(RadiansPerSecond), RadiansPerSecond.name());
    }

    @Override
    public void fromLog(LogTable table) {
      pivot.position.mut_replace(table.get("Pivot_Pos", pivot.position.in(Radians)), Radians);
      pivot.velocity.mut_replace(
          table.get("Pivot_Vel", pivot.velocity.in(RadiansPerSecond)), RadiansPerSecond);
      roller.velocity.mut_replace(
          table.get("Roller_Vel", roller.velocity.in(RadiansPerSecond)), RadiansPerSecond);
    }
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPivotVoltage(Voltage voltage) {}

  /** Set the roller speed with a value from -1 to 1. */
  public default void setRollerSpeed(double speed) {}
}
