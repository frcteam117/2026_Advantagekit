package frc.robot.util.components.simulators;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.States.PosVel_State;
import frc.robot.util.States.Pos_State;
import frc.robot.util.States.Vel_State;
import frc.robot.util.States.Volt_State;
import frc.robot.util.components.bases.ComponentSimControllerBase;
import frc.robot.util.components.bases.ComponentStates.Motor_State;
import frc.robot.util.mechanisms.MechanismConstants;
import java.util.Arrays;

public class DCMotorSimulator extends ComponentSimControllerBase {
  private final DCMotorSim sim;
  private final String[] componentNames;

  public DCMotorSimulator(DCMotorSim dcMotorSim, int... canIds) {
    componentNames = new String[canIds.length];
    for (int i = 0; i < componentNames.length; i++) {
      componentNames[i] = "CAN-" + canIds[i];
    }
    sim = dcMotorSim;
  }

  public DCMotorSimulator(MechanismConstants config) {
    // TODO: add the ability for the user to name the motors and make a more accurate DCMotorSim
    componentNames = new String[config.motorCanIds.length];
    for (int i = 0; i < componentNames.length; i++) {
      componentNames[i] = "CAN-" + config.motorCanIds[i];
    }
    sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(config.gearbox, config.moi_kgm2, config.reduction),
        config.gearbox);
  }

  public DCMotorSimulator(DCMotorSim dcMotorSim, String... motorNames) {
    componentNames = motorNames;
    sim = dcMotorSim;
  }

  public void setInput(Volt_State voltage_State) {
    sim.setInputVoltage(voltage_State.V());
  }

  @Override
  public void updateState(double deltaT_s) {
    sim.update(deltaT_s);
  }

  public void resetState(PosVel_State new_State) {
    sim.setAngle(new_State.pos());
    sim.setAngularVelocity(new_State.vel());
  }

  public void resetState(Pos_State new_State) {
    sim.setAngle(new_State.pos());
  }

  public void resetState(Vel_State new_State) {
    sim.setAngularVelocity(new_State.vel());
  }

  @Override
  public Motor_State[] getState() {
    Motor_State[] states = new Motor_State[componentNames.length];
    Arrays.fill(
        states,
        new Motor_State(
            sim.getAngularPositionRad() * sim.getGearing(),
            sim.getAngularVelocityRadPerSec() * sim.getGearing(),
            sim.getInputVoltage(),
            sim.getCurrentDrawAmps(),
            Double.NaN));
    return states;
  }

  @Override
  public String[] getComponentNames() {
    return componentNames;
  }

  @Override
  public String getControllerName() {
    return "DCMotorSimulator";
  }
}
