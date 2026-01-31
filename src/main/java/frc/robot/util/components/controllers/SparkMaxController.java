package frc.robot.util.components.controllers;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.States.Voltage_State;
import frc.robot.util.components.bases.ComponentControllerBase;
import frc.robot.util.components.bases.ComponentStates.Motor_State;

public class SparkMaxController extends ComponentControllerBase {
  private final SparkMax[] sparkMaxes;
  private final RelativeEncoder[] relativeEncoders;
  private final String[] componentNames;
  private final String controllerName;

  public SparkMaxController(SparkMaxControllerConfig config) {
    sparkMaxes = new SparkMax[config.canIds.length];
    relativeEncoders = new RelativeEncoder[config.canIds.length];
    componentNames = new String[config.canIds.length];
    controllerName = config.controllerName;

    for (int i = 0; i < config.canIds.length; i++) {
      componentNames[i] = "CAN-" + config.canIds[i];
      if (i >= 1) {
        config.baseSparkConfig.follow(config.canIds[0], config.followerInversions[i]);
      }
      if (config.motorNames == null) {
        sparkMaxes[i] = new SparkMax(config.canIds[i], config.baseMotorType);
      } else {
        sparkMaxes[i] = new SparkMax(config.canIds[i], config.motorTypes[i]);
      }
      if (config.sparkConfigs == null) {
        sparkMaxes[i].configure(config.baseSparkConfig, config.resetMode, config.persistMode);
      } else {
        sparkMaxes[i].configure(config.sparkConfigs[i], config.resetMode, config.persistMode);
      }
      relativeEncoders[i] = sparkMaxes[i].getEncoder();
    }
  }

  public void setInput(Voltage_State V_State) {
    sparkMaxes[0].setVoltage(V_State.V());
  }

  @Override
  public Motor_State[] getState() {
    Motor_State[] states = new Motor_State[sparkMaxes.length];
    for (int i = 0; i < sparkMaxes.length; i++) {
      states[i] = new Motor_State(
          relativeEncoders[i].getPosition() * 2 * Math.PI,
          relativeEncoders[i].getVelocity() * Math.PI / 30,
          sparkMaxes[i].getAppliedOutput() * sparkMaxes[i].getBusVoltage(),
          sparkMaxes[i].getOutputCurrent(),
          Double.NaN);
    }
    return states;
  }

  @Override
  public String[] getComponentNames() {
    return componentNames;
  }

  @Override
  public String getControllerName() {
    return controllerName;
  }

  public static class SparkMaxControllerConfig {
    public int[] canIds;
    public String[] motorNames;
    public String controllerName = "SparkMaxController";

    public MotorType baseMotorType;
    public MotorType[] motorTypes;
    public SparkMaxConfig baseSparkConfig;
    public SparkMaxConfig[] sparkConfigs;
    /** true if that motor should follow opposite of the first motor */
    public boolean[] followerInversions;

    public ResetMode resetMode = ResetMode.kResetSafeParameters;
    public PersistMode persistMode = PersistMode.kPersistParameters;
  }
}
