package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Unit;
import edu.wpi.first.util.struct.StructSerializable;

public class States {
  public interface StateBase {
    public abstract String getShortName();

    public abstract State getNaNState();
  }

  public static class StateValue<T> {
    private final T value;
    private final Class<T> clazz;
    private final String name;
    private final Unit unit;

    private StateValue(T value, Class<T> clazz, String name, Unit unit) {
      this.value = value;
      this.clazz = clazz;
      this.name = name;
      this.unit = unit;
    }

    public Class<T> getLoggableClass() {
      return clazz;
    }

    public T getValue() {
      return value;
    }

    public String getName() {
      return name;
    }

    public Unit getUnit() {
      return unit;
    }

    public static StateValue<Double> create(double value) {
      return new StateValue<Double>(value, Double.class, "", Value);
    }

    public static StateValue<Double> create(double value, String name) {
      return new StateValue<Double>(value, Double.class, name, Value);
    }

    public static StateValue<Double> create(double value, Unit unit) {
      return new StateValue<Double>(value, Double.class, "", unit);
    }

    public static StateValue<Double> create(Double value, String name, Unit unit) {
      return new StateValue<Double>(value, Double.class, name, unit);
    }

    public static StateValue<Boolean> create(boolean value) {
      return new StateValue<Boolean>(value, Boolean.class, "", Value);
    }

    public static StateValue<Boolean> create(boolean value, String name) {
      return new StateValue<Boolean>(value, Boolean.class, name, Value);
    }

    public static StateValue<Boolean> create(boolean value, Unit unit) {
      return new StateValue<Boolean>(value, Boolean.class, "", unit);
    }

    public static StateValue<Boolean> create(boolean value, String name, Unit unit) {
      return new StateValue<Boolean>(value, Boolean.class, name, unit);
    }

    public static StateValue<StructSerializable> create(StructSerializable value) {
      return new StateValue<StructSerializable>(value, StructSerializable.class, "", Value);
    }

    public static StateValue<StructSerializable> create(StructSerializable value, String name) {
      return new StateValue<StructSerializable>(value, StructSerializable.class, name, Value);
    }

    public static StateValue<StructSerializable> create(StructSerializable value, Unit unit) {
      return new StateValue<StructSerializable>(value, StructSerializable.class, "", unit);
    }

    public static StateValue<StructSerializable> create(
        StructSerializable value, String name, Unit unit) {
      return new StateValue<StructSerializable>(value, StructSerializable.class, name, unit);
    }
  }

  public interface State {
    public abstract StateValue<?>[] getValues();
  }

  public interface Linear {}

  public interface Angular {}

  public interface Pos_State extends State {
    public abstract double pos();

    public default double position() {
      return pos();
    }
  }

  public interface Vel_State extends State {
    public abstract double vel();

    public default double velocity() {
      return vel();
    }
  }

  public interface Acc_State extends State {
    public abstract double acc();

    public default double acceleration() {
      return acc();
    }
  }

  public interface Jerk_State extends State {
    public abstract double jerk();
  }

  public interface PosVel_State extends Pos_State, Vel_State {}

  public interface PosAcc_State extends Pos_State, Acc_State {}

  public interface PosJerk_State extends Pos_State, Jerk_State {}

  public interface VelAcc_State extends Vel_State, Acc_State {}

  public interface VelJerk_State extends Vel_State, Jerk_State {}

  public interface AccJerk_State extends Acc_State, Jerk_State {}

  public interface PosVelAcc_State extends PosVel_State, PosAcc_State, VelAcc_State {}

  public interface PosVelJerk_State extends PosVel_State, PosJerk_State, VelJerk_State {}

  public interface PosAccJerk_State extends PosAcc_State, PosJerk_State, AccJerk_State {}

  public interface VelAccJerk_State extends VelAcc_State, VelJerk_State, AccJerk_State {}

  public interface PosVelAccJerk_State
      extends PosVelAcc_State, PosVelJerk_State, PosAccJerk_State, VelAccJerk_State {}

  public static class Volt_State implements State {
    private final double V;
    private final String logName;

    public Volt_State(double V) {
      this(V, "");
    }

    public Volt_State(double V, String logName) {
      this.V = V;
      this.logName = logName;
    }

    public double V() {
      return V;
    }

    @Override
    public StateValue<?>[] getValues() {
      return new StateValue[] {StateValue.create(V, logName, Volts)};
    }
  }

  public static class Ampere_State implements State {
    private final double A;
    private final String logName;

    public Ampere_State(double A) {
      this(A, "");
    }

    public Ampere_State(double A, String logName) {
      this.A = A;
      this.logName = logName;
    }

    public double A() {
      return A;
    }

    @Override
    public StateValue<?>[] getValues() {
      return new StateValue[] {StateValue.create(A, logName, Amps)};
    }
  }

  public static class LinearP_State implements Linear, Pos_State {
    private final double m;
    private final String logName;

    public LinearP_State(double m) {
      this(m, "");
    }

    public LinearP_State(double m, String logName) {
      this.m = m;
      this.logName = logName;
    }

    public double m() {
      return m;
    }

    @Override
    public double pos() {
      return m;
    }

    @Override
    public StateValue<?>[] getValues() {
      return new StateValue[] {StateValue.create(m, logName, Meters)};
    }
  }

  // public record LinearV_State(double mPs) implements Linear, Vel_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {MetersPerSecond};
  //   }

  //   public double vel() {
  //     return this.mPs();
  //   }
  // }

  // public record LinearA_State(double mPs2) implements Linear, Acc_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {MetersPerSecondPerSecond};
  //   }

  //   public double acc() {
  //     return this.mPs2();
  //   }
  // }

  // public record LinearJ_State(double mPs3) implements Linear, Jerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {MetersPerSecondPerSecond.per(Second)};
  //   }

  //   public double jerk() {
  //     return this.mPs3();
  //   }
  // }

  public static class LinearPV_State implements Linear, PosVel_State {
    private final double m;
    private final double mPs;
    private final String[] logNames;

    public LinearPV_State(double m, double mPs) {
      this(m, mPs, "", "");
    }

    public LinearPV_State(double m, double mPs, String... logNames) {
      this.m = m;
      this.mPs = mPs;
      this.logNames = logNames;
    }

    public double m() {
      return m;
    }

    public double mPs() {
      return mPs;
    }

    @Override
    public double pos() {
      return m;
    }

    @Override
    public double vel() {
      return mPs;
    }

    @Override
    public StateValue<?>[] getValues() {
      return new StateValue[] {
        StateValue.create(m, logNames[0], Meters),
        StateValue.create(mPs, logNames[1], MetersPerSecond)
      };
    }
  }

  // public record LinearPA_State(double m, double mPs2) implements Linear, PosAcc_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {Meters, MetersPerSecondPerSecond};
  //   }

  //   public double pos() {
  //     return this.m();
  //   }

  //   public double acc() {
  //     return this.mPs2();
  //   }
  // }

  // public record LinearPJ_State(double m, double mPs3) implements Linear, PosJerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {Meters, MetersPerSecondPerSecond.per(Second)};
  //   }

  //   public double pos() {
  //     return this.m();
  //   }

  //   public double jerk() {
  //     return this.mPs3();
  //   }
  // }

  // public record LinearVA_State(double mPs, double mPs2) implements Linear, VelAcc_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {MetersPerSecond, MetersPerSecondPerSecond};
  //   }

  //   public double vel() {
  //     return this.mPs();
  //   }

  //   public double acc() {
  //     return this.mPs2();
  //   }
  // }

  // public record LinearVJ_State(double mPs, double mPs3) implements Linear, VelJerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {MetersPerSecond, MetersPerSecondPerSecond.per(Second)};
  //   }

  //   public double vel() {
  //     return this.mPs();
  //   }

  //   public double jerk() {
  //     return this.mPs3();
  //   }
  // }

  // public record LinearAJ_State(double mPs2, double mPs3) implements Linear, AccJerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {MetersPerSecondPerSecond, MetersPerSecondPerSecond.per(Second)};
  //   }

  //   public double acc() {
  //     return this.mPs2();
  //   }

  //   public double jerk() {
  //     return this.mPs3();
  //   }
  // }

  // public record LinearPVA_State(double m, double mPs, double mPs2)
  //     implements Linear, PosVelAcc_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {Meters, MetersPerSecond, MetersPerSecondPerSecond};
  //   }

  //   public double pos() {
  //     return this.m();
  //   }

  //   public double vel() {
  //     return this.mPs();
  //   }

  //   public double acc() {
  //     return this.mPs2();
  //   }
  // }

  // public record LinearPVJ_State(double m, double mPs, double mPs3)
  //     implements Linear, PosVelJerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {Meters, MetersPerSecond, MetersPerSecondPerSecond.per(Second)};
  //   }

  //   public double pos() {
  //     return this.m();
  //   }

  //   public double vel() {
  //     return this.mPs();
  //   }

  //   public double jerk() {
  //     return this.mPs3();
  //   }
  // }

  // public record LinearPAJ_State(double m, double mPs2, double mPs3)
  //     implements Linear, PosAccJerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {Meters, MetersPerSecondPerSecond, MetersPerSecondPerSecond.per(Second)};
  //   }

  //   public double pos() {
  //     return this.m();
  //   }

  //   public double acc() {
  //     return this.mPs2();
  //   }

  //   public double jerk() {
  //     return this.mPs3();
  //   }
  // }

  // public record LinearVAJ_State(double mPs, double mPs2, double mPs3)
  //     implements Linear, VelAccJerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {
  //       MetersPerSecond, MetersPerSecondPerSecond, MetersPerSecondPerSecond.per(Second)
  //     };
  //   }

  //   public double vel() {
  //     return this.mPs();
  //   }

  //   public double acc() {
  //     return this.mPs2();
  //   }

  //   public double jerk() {
  //     return this.mPs3();
  //   }
  // }

  // public record LinearPVAJ_State(double m, double mPs, double mPs2, double mPs3)
  //     implements Linear, PosVelAccJerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {
  //       Meters, MetersPerSecond, MetersPerSecondPerSecond, MetersPerSecondPerSecond.per(Second)
  //     };
  //   }

  //   public double pos() {
  //     return this.m();
  //   }

  //   public double vel() {
  //     return this.mPs();
  //   }

  //   public double acc() {
  //     return this.mPs2();
  //   }

  //   public double jerk() {
  //     return this.mPs3();
  //   }
  // }

  public static class AngularP_State implements Angular, Pos_State {
    private final double rad;
    private final String logName;

    public AngularP_State(double rad) {
      this(rad, "");
    }

    public AngularP_State(double rad, String logName) {
      this.rad = rad;
      this.logName = logName;
    }

    public double rad() {
      return rad;
    }

    @Override
    public double pos() {
      return rad;
    }

    @Override
    public StateValue<?>[] getValues() {
      return new StateValue[] {StateValue.create(rad, logName, Radians)};
    }
  }

  public static class AngularV_State implements Angular, Vel_State {
    private final double radPs;
    private final String logName;

    public AngularV_State(double radPs) {
      this(radPs, "");
    }

    public AngularV_State(double radPs, String logName) {
      this.radPs = radPs;
      this.logName = logName;
    }

    public double rad() {
      return radPs;
    }

    @Override
    public double vel() {
      return radPs;
    }

    @Override
    public StateValue<?>[] getValues() {
      return new StateValue[] {StateValue.create(radPs, logName, RadiansPerSecond)};
    }
  }

  // public record AngularA_State(double radPs2) implements Angular, Acc_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {RadiansPerSecondPerSecond};
  //   }

  //   public double acc() {
  //     return this.radPs2();
  //   }
  // }

  // public record AngularJ_State(double radPs3) implements Angular, Jerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {RadiansPerSecondPerSecond.per(Second)};
  //   }

  //   public double jerk() {
  //     return this.radPs3();
  //   }
  // }

  public static class AngularPV_State implements Angular, PosVel_State {
    private final double rad;
    private final double radPs;
    private final String[] logNames;

    public AngularPV_State(double rad, double radPs) {
      this(rad, radPs, "", "");
    }

    public AngularPV_State(double rad, double radPs, String... logNames) {
      this.rad = rad;
      this.radPs = radPs;
      this.logNames = logNames;
    }

    public double rad() {
      return rad;
    }

    public double radPs() {
      return radPs;
    }

    @Override
    public double pos() {
      return rad;
    }

    @Override
    public double vel() {
      return radPs;
    }

    @Override
    public StateValue<?>[] getValues() {
      return new StateValue[] {
        StateValue.create(rad, logNames[0], Radians),
        StateValue.create(radPs, logNames[1], RadiansPerSecond)
      };
    }
  }

  // public record AngularPA_State(double rad, double radPs2) implements Angular, PosAcc_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {Radians, RadiansPerSecondPerSecond};
  //   }

  //   public double pos() {
  //     return this.rad();
  //   }

  //   public double acc() {
  //     return this.radPs2();
  //   }
  // }

  // public record AngularPJ_State(double rad, double radPs3) implements Angular, PosJerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {Radians, RadiansPerSecondPerSecond.per(Second)};
  //   }

  //   public double pos() {
  //     return this.rad();
  //   }

  //   public double jerk() {
  //     return this.radPs3();
  //   }
  // }

  public class AngularVA_State implements Angular, VelAcc_State {
    private final double radPs;
    private final double radPs2;
    private final String[] logNames;

    public AngularVA_State(double radPs, double radPs2) {
      this(radPs, radPs2, "", "");
    }

    public AngularVA_State(double radPs, double radPs2, String... logNames) {
      this.radPs = radPs;
      this.radPs2 = radPs2;
      this.logNames = logNames;
    }

    public double rad() {
      return radPs;
    }

    public double radPs() {
      return radPs2;
    }

    @Override
    public double vel() {
      return radPs;
    }

    @Override
    public double acc() {
      return radPs2;
    }

    @Override
    public StateValue<?>[] getValues() {
      return new StateValue[] {
        StateValue.create(radPs, logNames[0], RadiansPerSecond),
        StateValue.create(radPs2, logNames[1], RadiansPerSecondPerSecond)
      };
    }
  }

  // public record AngularVJ_State(double radPs, double radPs3) implements Angular, VelJerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {RadiansPerSecond, RadiansPerSecondPerSecond.per(Second)};
  //   }

  //   public double vel() {
  //     return this.radPs();
  //   }

  //   public double jerk() {
  //     return this.radPs3();
  //   }
  // }

  // public record AngularAJ_State(double radPs2, double radPs3) implements Angular, AccJerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {RadiansPerSecondPerSecond, RadiansPerSecondPerSecond.per(Second)};
  //   }

  //   public double acc() {
  //     return this.radPs2();
  //   }

  //   public double jerk() {
  //     return this.radPs3();
  //   }
  // }

  // public record AngularPVA_State(double rad, double radPs, double radPs2)
  //     implements Angular, PosVelAcc_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {Radians, RadiansPerSecond, RadiansPerSecondPerSecond};
  //   }

  //   public double pos() {
  //     return this.rad();
  //   }

  //   public double vel() {
  //     return this.radPs();
  //   }

  //   public double acc() {
  //     return this.radPs2();
  //   }
  // }

  // public record AngularPVJ_State(double rad, double radPs, double radPs3)
  //     implements Angular, PosVelJerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {Radians, RadiansPerSecond, RadiansPerSecondPerSecond.per(Second)};
  //   }

  //   public double pos() {
  //     return this.rad();
  //   }

  //   public double vel() {
  //     return this.radPs();
  //   }

  //   public double jerk() {
  //     return this.radPs3();
  //   }
  // }

  // public record AngularPAJ_State(double rad, double radPs2, double radPs3)
  //     implements Angular, PosAccJerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {Radians, RadiansPerSecondPerSecond,
  // RadiansPerSecondPerSecond.per(Second)};
  //   }

  //   public double pos() {
  //     return this.rad();
  //   }

  //   public double acc() {
  //     return this.radPs2();
  //   }

  //   public double jerk() {
  //     return this.radPs3();
  //   }
  // }

  // public record AngularVAJ_State(double radPs, double radPs2, double radPs3)
  //     implements Angular, VelAccJerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {
  //       RadiansPerSecond, RadiansPerSecondPerSecond, RadiansPerSecondPerSecond.per(Second)
  //     };
  //   }

  //   public double vel() {
  //     return this.radPs();
  //   }

  //   public double acc() {
  //     return this.radPs2();
  //   }

  //   public double jerk() {
  //     return this.radPs3();
  //   }
  // }

  // public record AngularPVAJ_State(double rad, double radPs, double radPs2, double radPs3)
  //     implements Angular, PosVelAccJerk_State {
  //   @Override
  //   public Unit[] getUnits() {
  //     return new Unit[] {
  //       Radians, RadiansPerSecond, RadiansPerSecondPerSecond,
  // RadiansPerSecondPerSecond.per(Second)
  //     };
  //   }

  //   public double pos() {
  //     return this.rad();
  //   }

  //   public double vel() {
  //     return this.radPs();
  //   }

  //   public double acc() {
  //     return this.radPs2();
  //   }

  //   public double jerk() {
  //     return this.radPs3();
  //   }
  // }
}
