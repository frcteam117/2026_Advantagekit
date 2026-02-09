package frc.robot.util.components.simulators;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.States.PosVel_State;
import frc.robot.util.States.Pos_State;
import frc.robot.util.States.Vel_State;
import frc.robot.util.States.Voltage_State;
import frc.robot.util.components.bases.ComponentSimControllerBase;
import frc.robot.util.components.bases.ComponentStates.Motor_State;
import frc.robot.util.mechanisms.MechanismConstants;
import java.util.Arrays;

public class ArmSimulator extends ComponentSimControllerBase {
  private final SingleJointedArmSim sim;
  private final String[] motorNames;
  private final double reduction;
  private final double cmOffset_rad;
  private double motorVoltage_V = 0;

  public ArmSimulator(ArmSimulatorConfig config) {
    if (config.motorNames == null) {
      config.motorNames = new String[config.canIds.length];
      for (int i = 0; i < config.motorNames.length; i++) {
        config.motorNames[i] = "CAN-" + config.canIds[i];
      }
    }
    if (config.length_m == 0) {
      config.length_m = Math.sqrt(3
          * config.moi_kgm2
          / config
              .mass_kg); // SingleJointedArmSim assumes J=1/3 mL² -> L=(3J/m)^½, m=mass, L=length,
      // J=moi
    }
    motorNames = config.motorNames;
    sim = new SingleJointedArmSim(
        config.plant,
        config.gearbox,
        config.reduction,
        config.length_m,
        config.min_rad + config.cmOffset_rad,
        config.max_rad + config.cmOffset_rad,
        true,
        config.start_rad + config.cmOffset_rad);
    reduction = config.reduction;
    cmOffset_rad = config.cmOffset_rad;
  }

  public ArmSimulator(MechanismConstants config) {
    if (config.motorNames == null) {
      config.motorNames = new String[config.motorCanIds.length];
      for (int i = 0; i < config.motorNames.length; i++) {
        config.motorNames[i] = "CAN-" + config.motorCanIds[i];
      }
    }
    if (config.length_m == null) {
      config.length_m = Math.sqrt(3
          * config.moi_kgm2
          / config
              .mass_kg); // SingleJointedArmSim assumes J=1/3 mL² -> L=(3J/m)^½, m=mass, L=length,
      // J=moi
    }
    motorNames = config.motorNames;
    reduction = config.reduction;
    cmOffset_rad = config.cmOffset_Pos.pos();
    // TODO: allow making your own plant
    sim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            config.gearbox, config.moi_kgm2, config.reduction),
        config.gearbox,
        config.reduction,
        config.length_m,
        config.min_Pos.pos() + config.cmOffset_Pos.pos(),
        config.max_Pos.pos() + config.cmOffset_Pos.pos(),
        true,
        ((Pos_State) config.start_State).pos() + config.cmOffset_Pos.pos());
  }

  public void setInput(Voltage_State voltage_State) {
    motorVoltage_V = voltage_State.V();
    sim.setInputVoltage(motorVoltage_V);
  }

  @Override
  public void updateState(double deltaT_s) {
    sim.update(deltaT_s);
  }

  public void resetState(PosVel_State new_State) {
    sim.setState(new_State.pos(), new_State.vel());
  }

  public void resetState(Pos_State new_State) {
    sim.setState(new_State.pos(), sim.getVelocityRadPerSec());
  }

  public void resetState(Vel_State new_State) {
    sim.setState(sim.getAngleRads(), new_State.vel());
  }

  @Override
  public Motor_State[] getState() {
    Motor_State[] states = new Motor_State[motorNames.length];
    Arrays.fill(
        states,
        new Motor_State(
            (sim.getAngleRads() - cmOffset_rad) * reduction,
            sim.getVelocityRadPerSec() * reduction,
            motorVoltage_V,
            sim.getCurrentDrawAmps(),
            Double.NaN));
    return states;
  }

  @Override
  public String[] getComponentNames() {
    return motorNames;
  }

  @Override
  public String getControllerName() {
    return "ArmSimulator";
    // return config.controllerName;
  }

  public static class ArmSimulatorConfig {
    public LinearSystem<N2, N1, N2> plant;
    public DCMotor gearbox;
    public double reduction;
    /** Use moi and mass instead to make simulation more accurate. */
    public double length_m;

    public double moi_kgm2;
    public double mass_kg;
    public double min_rad;
    public double max_rad;
    public double start_rad;
    public double posStdDev = 0;
    /** Angle of the center of mass above horizontal when the measured angle is zero */
    public double cmOffset_rad;

    public int[] canIds;
    public String[] motorNames;
    public String controllerName = "DCMotorSimulator";
  }
}
