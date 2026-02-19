package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.PivotConstants.HardwareConstants;
import frc.robot.subsystems.intake.PivotConstants.MechanismConstants;
import frc.robot.subsystems.intake.PivotConstants.PIDConstants;
import frc.robot.subsystems.intake.PivotConstants.ProfileConstants;
import frc.robot.subsystems.intake.PivotConstants.SafetyConstants;
import frc.robot.subsystems.intake.PivotConstants.TelemetryConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

public class Pivot extends SubsystemBase {
  public Pivot() {
    spark = new SparkMax(HardwareConstants.canId, HardwareConstants.motorType);
    // TODO: change "spark" to "pivotMotor"
    sparkSmartMotorController = new SparkWrapper(spark, HardwareConstants.motor, smcConfig);
    armCfg = new ArmConfig(sparkSmartMotorController)
        // Soft limit is applied to the SmartMotorControllers PID
        .withSoftLimits(MechanismConstants.kMinSoftLimit, MechanismConstants.kMaxSoftLimit)
        // Hard limit is applied to the simulation.
        .withHardLimit(MechanismConstants.kMinHardLimit, MechanismConstants.kMaxHardLimit)
        // Starting position is where your arm starts
        .withStartingPosition(MechanismConstants.kStartingPosition)
        // Length and mass of your arm for sim.
        .withLength(MechanismConstants.kArmLength)
        .withMass(MechanismConstants.kArmMass)
        // Telemetry name and verbosity for the arm.
        .withTelemetry(TelemetryConstants.kArmName, TelemetryConstants.kTelemetryVerbosity);
    arm = new Arm(armCfg);
  }

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(PIDConstants.kS, PIDConstants.kG, PIDConstants.kV, PIDConstants.kA);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(HardwareConstants.kControlMode)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(
          PIDConstants.sparkP,
          PIDConstants.sparkI,
          PIDConstants.sparkD,
          ProfileConstants.kSparkMaxVelocity,
          ProfileConstants.kSparkMaxAcceleration)
      .withSimClosedLoopController(
          PIDConstants.sparkP,
          PIDConstants.sparkI,
          PIDConstants.sparkD,
          ProfileConstants.kSparkMaxVelocity,
          ProfileConstants.kSparkMaxAcceleration)
      // Feedforward Constants
      // Telemetry name and verbosity level
      .withTelemetry(TelemetryConstants.kMotorName, TelemetryConstants.kTelemetryVerbosity)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(
          MechanismConstants.kGearRatioStage1, MechanismConstants.kGearRatioStage2)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(HardwareConstants.kInverted)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(SafetyConstants.kCurrentLimit)
      .withClosedLoopRampRate(SafetyConstants.kRampRate)
      .withOpenLoopRampRate(SafetyConstants.kRampRate);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(
      PIDConstants.rioP,
      PIDConstants.rioI,
      PIDConstants.rioD,
      new TrapezoidProfile.Constraints(
          ProfileConstants.kRioMaxVelocity, ProfileConstants.kRioMaxAcceleration));
  private SparkMax spark; // = new SparkMax(HardwareConstants.canId, HardwareConstants.motorType);
  private SmartMotorController sparkSmartMotorController; // sparkSmartMotorController =
  // new SparkWrapper(spark, HardwareConstants.motor, smcConfig);

  private ArmConfig armCfg; // = new ArmConfig(sparkSmartMotorController)
  //// Soft limit is applied to the SmartMotorControllers PID
  // .withSoftLimits(MechanismConstants.kMinSoftLimit, MechanismConstants.kMaxSoftLimit)
  // Hard limit is applied to the simulation.
  // .withHardLimit(MechanismConstants.kMinHardLimit, MechanismConstants.kMaxHardLimit)
  // Starting position is where your arm starts
  // .withStartingPosition(MechanismConstants.kStartingPosition)
  // Length and mass of your arm for sim.
  // .withLength(MechanismConstants.kArmLength)
  // .withMass(MechanismConstants.kArmMass)
  // Telemetry name and verbosity for the arm.
  // .withTelemetry(TelemetryConstants.kArmName, TelemetryConstants.kTelemetryVerbosity);
  private Arm arm; // = new Arm(armCfg);

  public Command setAngle(Angle angle) {
    return setAngleWithRioProfile(angle);
  }

  /**
   * Set the angle of the arm using the RIO-side ProfiledPIDController.
   * @param angle Target angle.
   * @return Command to run.
   */
  public Command setAngleWithRioProfile(Angle angle) {
    return run(() -> {
          double pidOutput = m_controller.calculate(arm.getAngle().in(Radians), angle.in(Radians));
          double ffOutput = m_feedforward.calculate(
              m_controller.getSetpoint().position, m_controller.getSetpoint().velocity);
          spark.setVoltage(pidOutput + ffOutput);
        })
        .beforeStarting(() -> m_controller.reset(arm.getAngle().in(Radians)));
  }

  @Override
  public void periodic() {
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }
}
