package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.BottomHopperConstants.HardwareConstants;
import frc.robot.subsystems.indexer.BottomHopperConstants.MechanismConstants;
import frc.robot.subsystems.indexer.BottomHopperConstants.PIDConstants;
import frc.robot.subsystems.indexer.BottomHopperConstants.SafetyConstants;
import frc.robot.subsystems.indexer.BottomHopperConstants.TelemetryConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class BottomHopper extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(HardwareConstants.kControlMode)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(
          PIDConstants.controllerP, PIDConstants.controllerI, PIDConstants.controllerD)
      .withSimClosedLoopController(
          PIDConstants.controllerP, PIDConstants.controllerI, PIDConstants.controllerD)
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(
          PIDConstants.feedForwardS, PIDConstants.feedForwardV, PIDConstants.feedForwardA))
      .withSimFeedforward(new SimpleMotorFeedforward(
          PIDConstants.feedForwardS, PIDConstants.feedForwardV, PIDConstants.feedForwardA))
      // Telemetry name and verbosity level
      .withTelemetry(TelemetryConstants.kMotorName, TelemetryConstants.kTelemetryVerbosity)
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as
      // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(
          MechanismConstants.kGearRatioStage1, MechanismConstants.kGearRatioStage2)))
      // Motor properties to prevent over currenting.
      .withOpenLoopRampRate(
          SafetyConstants.kRampRate) // Time in seconds to go from 0 to full speed.
      .withMotorInverted(HardwareConstants.kInverted)
      .withIdleMode(HardwareConstants.kIdleMode)
      .withStatorCurrentLimit(SafetyConstants.kCurrentLimit);
  // Vendor motor controller object
  private SparkMax spark = new SparkMax(HardwareConstants.canId, HardwareConstants.motorType);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController =
      new SparkWrapper(spark, HardwareConstants.motor, smcConfig);
  private final FlyWheelConfig hopperConfig = new FlyWheelConfig(sparkSmartMotorController)
      // Diameter of the flywheel.
      .withDiameter(MechanismConstants.kDiameter)
      // Mass of the flywheel.
      .withMass(MechanismConstants.kMass)
      // Maximum speed of the hopper.
      .withUpperSoftLimit(MechanismConstants.kMaxSpeed)
      // Telemetry name and verbosity for the arm.
      .withTelemetry(TelemetryConstants.kHopperName, TelemetryConstants.kHopperTelemetryVerbosity);

  // hopper Mechanism
  private FlyWheel hopper = new FlyWheel(hopperConfig);

  /**
   * Gets the current velocity of the hopper.
   *
   * @return hopper velocity.
   */
  public AngularVelocity getVelocity() {
    return hopper.getSpeed();
  }

  /**
   * Set the hopper velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return hopper.setSpeed(speed);
  }

  /**
   * Set the dutycycle of the hopper.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return hopper.set(dutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hopper.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    hopper.simIterate();
  }
}
