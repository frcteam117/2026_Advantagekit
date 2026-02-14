package frc.robot.util.components.controllers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.util.components.bases.ComponentControllerBase;
import frc.robot.util.components.bases.ComponentStates.Motor_State;
import frc.robot.util.mechanisms.MechanismConstants;
import frc.robot.util.states.Voltage_State;

public class SparkMaxController extends ComponentControllerBase {
  private final SparkMax[] sparkMaxes;
  private final RelativeEncoder[] relativeEncoders;
  private final String[] componentNames;
  private final String controllerName;

  public SparkMaxController(MechanismConstants<?> config) {
    sparkMaxes = new SparkMax[config.motorCanIds.length];
    relativeEncoders = new RelativeEncoder[config.motorCanIds.length];
    componentNames = new String[config.motorCanIds.length];
    controllerName = config.outputsLogName;

    for (int i = 0; i < config.motorCanIds.length; i++) {
      componentNames[i] = "CAN-" + config.motorCanIds[i];
      if (i >= 1) {
        config.baseSparkConfig.follow(config.motorCanIds[0], config.followerInversions[i]);
      }
      if (config.revMotorTypes == null) {
        sparkMaxes[i] = new SparkMax(config.motorCanIds[i], config.revMotorType);
      } else {
        sparkMaxes[i] = new SparkMax(config.motorCanIds[i], config.revMotorTypes[i]);
      }
      if (config.sparkConfigs == null) {
        sparkMaxes[i].configure(config.baseSparkConfig, config.revResetMode, config.revPersistMode);
      } else {
        sparkMaxes[i].configure(config.sparkConfigs[i], config.revResetMode, config.revPersistMode);
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
}
