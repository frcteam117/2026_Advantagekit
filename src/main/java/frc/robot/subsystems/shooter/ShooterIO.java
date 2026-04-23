package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ShooterIO {
  public static class ShooterIOInputs implements LoggableInputs {
    public final MutAngle hood_Pos = Radians.mutable(0);
    public final MutAngularVelocity hood_Vel = RadiansPerSecond.mutable(0);
    public final MutAngularVelocity rioFlywheel_Vel = RadiansPerSecond.mutable(0);
    public final MutAngularVelocity pdhFlywheel_Vel = RadiansPerSecond.mutable(0);
    public double rioFlywheel_AppliedOutput = 0;
    public double pdhFlywheel_AppliedOutput = 0;

    @Override
    public void toLog(LogTable table) {
      table.put("Hood_Pos", hood_Pos.magnitude(), hood_Pos.unit().name());
      table.put("Hood_Vel", hood_Vel.magnitude(), hood_Vel.unit().name());
      table.put(
          "RioFlywheel_Vel", rioFlywheel_Vel.magnitude(), rioFlywheel_Vel.unit().name());
      table.put(
          "PdhFlywheel_Vel", pdhFlywheel_Vel.magnitude(), pdhFlywheel_Vel.unit().name());
      table.put("RioFlywheel_AppliedOutput", rioFlywheel_AppliedOutput, Volts.name());
      table.put("PdhFlywheel_AppliedOutput", pdhFlywheel_AppliedOutput, Volts.name());
    }

    @Override
    public void fromLog(LogTable table) {
      hood_Pos.mut_setMagnitude(table.get("Hood_Pos", hood_Pos.magnitude()));
      hood_Vel.mut_setMagnitude(table.get("Hood_Vel", hood_Vel.magnitude()));
      rioFlywheel_Vel.mut_setMagnitude(table.get("RioFlywheel_Vel", rioFlywheel_Vel.magnitude()));
      pdhFlywheel_Vel.mut_setMagnitude(table.get("PdhFlywheel_Vel", pdhFlywheel_Vel.magnitude()));
      rioFlywheel_AppliedOutput = table.get("RioFlywheel_AppliedOutput", rioFlywheel_AppliedOutput);
      rioFlywheel_AppliedOutput = table.get("PdhFlywheel_AppliedOutput", pdhFlywheel_AppliedOutput);
    }
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setHoodVoltage(Voltage voltage) {}

  public default void setRIOFlywheelVoltage(Voltage voltage) {}

  public default void setPDHFlywheelVoltage(Voltage voltage) {}
}
