package frc.robot.util.components.bases;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.util.StateUtil.State;
import frc.robot.util.StateUtil.StateValue;

public class ComponentStates {

  public static interface ComponentState extends State {}

  public static class AbsoluteEncoder_State implements ComponentState {
    private final double rad;
    private final String logName;

    public AbsoluteEncoder_State(double rad) {
      this(rad, "Abs");
    }

    public AbsoluteEncoder_State(double rad, String logName) {
      this.rad = rad;
      this.logName = logName;
    }

    public double rad() {
      return rad;
    }

    @Override
    public StateValue[] getValues() {
      return new StateValue[] {new StateValue(rad, logName, Radians)};
    }
  }

  public static class Motor_State implements ComponentState {
    private final double[] values;
    private final String[] logNames;

    public Motor_State(double rad, double radPs, double motor_V, double motor_A, double supply_A) {
      this(rad, radPs, motor_V, motor_A, supply_A, "Motor", "Motor", "Motor", "Motor", "Supply");
    }

    public Motor_State(
        double rad,
        double radPs,
        double motor_V,
        double motor_A,
        double supply_A,
        String... logNames) {
      this.values = new double[] {rad, radPs, motor_V, motor_A, supply_A};
      this.logNames = logNames;
    }

    public double rad() {
      return values[0];
    }

    public double radPs() {
      return values[1];
    }

    public double motor_V() {
      return values[2];
    }

    public double motor_A() {
      return values[3];
    }

    public double supply_A() {
      return values[4];
    }

    @Override
    public StateValue[] getValues() {
      return new StateValue[] {
        new StateValue(values[0], logNames[0], Radians),
        new StateValue(values[1], logNames[1], RadiansPerSecond),
        new StateValue(values[2], logNames[2], Volts),
        new StateValue(values[3], logNames[3], Amps),
        new StateValue(values[4], logNames[4], Amps)
      };
    }
  }

  public static record AngularMechanismState(double rad, double radPs, Motor_State... motors) {
    public AngularMechanismState(double rad, double radPs, Motor_State motor) {
      this(rad, radPs, new Motor_State[] {motor});
    }

    public AngularMechanismState(double rad, double radPs) {
      this(rad, radPs, new Motor_State[0]);
    }
  }

  public static record LinearMechanismState(
      double m, double mPs, double motor_V, double motor_A, double supply_A) {}

  // public static record MechanismState<T extends State>(T mechanism, Motor_State... components) {
  //   public static ComponentState[] ra =
  //       new ComponentState[] {new Motor_State(0, 0, 0), new AbsoluteEncoder_State(0)};

  // static {
  //   double s = ((MotorState) ra[1]).motor_A;
  // }
  // }
}
