package frc.robot.util.components.bases;

public class ComponentStates {

  public static interface ComponentState {}

  public static record AbsoluteEncoder_State(double rad) implements ComponentState {}

  public static record Motor_State(
      double rad, double radPs, double motor_V, double motor_A, double supply_A)
      implements ComponentState {}

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
