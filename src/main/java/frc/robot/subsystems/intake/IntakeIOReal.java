package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeIOReal implements IntakeIO {
  private final SparkMax pivot_Spark = new SparkMax(Pivot.CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder pivot_Encoder = pivot_Spark.getEncoder();

  private final SparkFlex roller_Spark = new SparkFlex(Roller.CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder roller_Encoder = roller_Spark.getEncoder();

  public IntakeIOReal() {
    pivot_Spark.configure(
        Pivot.SPARK_MAX_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    roller_Spark.configure(
        Roller.SPARK_FLEX_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivot.position.mut_replace(
        pivot_Encoder.getPosition() * 2 * Math.PI / Pivot.REDUCTION, Radians);
    inputs.pivot.velocity.mut_replace(
        pivot_Encoder.getVelocity() * 2 * Math.PI / (Pivot.REDUCTION * 60), RadiansPerSecond);
    inputs.pivot.outputCurrent.mut_replace(pivot_Spark.getOutputCurrent(), Amps);
    inputs.roller.velocity.mut_replace(
        roller_Encoder.getVelocity() * 2 * Math.PI / (Roller.REDUCTION * 60), RadiansPerSecond);
  }

  @Override
  public void setPivotVoltage(Voltage voltage) {
    pivot_Spark.setVoltage(voltage);
  }

  @Override
  public void resetPivotPosition(Angle position) {
    pivot_Encoder.setPosition(position.in(Radians));
  }

  @Override
  public void setRollerSpeed(double speed) {
    roller_Spark.set(speed);
  }
}
