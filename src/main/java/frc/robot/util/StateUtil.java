package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.logging.LogUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

public class StateUtil {
  public static String toSuffix(StateValue value) {
    return value.getName() + LogUtil.toSuffix(value.getUnit().symbol());
  }

  public static void recordOutput(String key, State state) {
    recordOutput(key, state.getValues());
  }

  public static void recordNullOutput(String key, State state) {
    recordOutput(key, state.getValues());
  }

  public static void overrideOutput(String key, State newState, State oldState) {
    overrideOutput(key, newState.getValues(), oldState.getValues());
  }

  public static void overrideWithNullOutput(String key, State newState, State oldState) {
    overrideWithNullOutput(key, newState.getValues(), oldState.getValues());
  }

  public static void recordOutput(String key, StateValue... values) {
    for (StateValue value : values) {
      recordOutput(key + toSuffix(value), value);
    }
  }

  public static void recordNullOutput(String key, StateValue... values) {
    for (StateValue value : values) {
      recordNullOutput(key + toSuffix(value), value);
    }
  }

  public static void overrideOutput(String key, StateValue[] newValues, StateValue[] oldValues) {
    for (int i = 0; i < Math.max(newValues.length, oldValues.length); i++) {
      if (i < Math.min(newValues.length, oldValues.length)) {
        String newKey = key + toSuffix(newValues[i]);
        String oldKey = key + toSuffix(oldValues[i]);
        if (newKey != oldKey) {
          recordNullOutput(oldKey, oldValues[i]);
        }
        recordOutput(newKey, newValues[i]);
      } else if (i < newValues.length) {
        recordOutput(key + toSuffix(newValues[i]), newValues[i]);
      } else if (i < oldValues.length) {
        recordNullOutput(key + toSuffix(oldValues[i]), oldValues[i]);
      }
    }
  }

  public static void overrideWithNullOutput(
      String key, StateValue[] newValues, StateValue[] oldValues) {
    for (int i = 0; i < Math.max(newValues.length, oldValues.length); i++) {
      if (i < Math.min(newValues.length, oldValues.length)) {
        String newKey = key + toSuffix(newValues[i]);
        String oldKey = key + toSuffix(oldValues[i]);
        if (newKey != oldKey) {
          recordNullOutput(oldKey, oldValues[i]);
        }
        recordNullOutput(newKey, newValues[i]);
      } else if (i < newValues.length) {
        recordNullOutput(key + toSuffix(newValues[i]), newValues[i]);
      } else if (i < oldValues.length) {
        recordNullOutput(key + toSuffix(oldValues[i]), oldValues[i]);
      }
    }
  }

  @SuppressWarnings("unchecked")
  private static void recordOutput(String newKey, StateValue value) {
    if (value.getValue() instanceof byte[]) {
      Logger.recordOutput(newKey, (byte[]) value.getValue());
    } else if (value.getValue() instanceof byte[][]) {
      Logger.recordOutput(newKey, (byte[][]) value.getValue());
    } else if (value.getValue() instanceof Boolean) {
      Logger.recordOutput(newKey, (Boolean) value.getValue());
    } else if (value.getValue() instanceof BooleanSupplier) {
      Logger.recordOutput(newKey, (BooleanSupplier) value.getValue());
    } else if (value.getValue() instanceof boolean[]) {
      Logger.recordOutput(newKey, (boolean[]) value.getValue());
    } else if (value.getValue() instanceof boolean[][]) {
      Logger.recordOutput(newKey, (boolean[][]) value.getValue());
    } else if (value.getValue() instanceof Integer) {
      if (value.getUnit() == null) {
        Logger.recordOutput(newKey, (Integer) value.getValue());
      } else {
        Logger.recordOutput(newKey, (Integer) value.getValue(), value.getUnit());
      }
    } else if (value.getValue() instanceof IntSupplier) {
      Logger.recordOutput(newKey, (IntSupplier) value.getValue());
    } else if (value.getValue() instanceof int[]) {
      Logger.recordOutput(newKey, (int[]) value.getValue());
    } else if (value.getValue() instanceof int[][]) {
      Logger.recordOutput(newKey, (int[][]) value.getValue());
    } else if (value.getValue() instanceof Long) {
      if (value.getUnit() == null) {
        Logger.recordOutput(newKey, (Long) value.getValue());
      } else {
        Logger.recordOutput(newKey, (Long) value.getValue(), value.getUnit());
      }
    } else if (value.getValue() instanceof LongSupplier) {
      Logger.recordOutput(newKey, (LongSupplier) value.getValue());
    } else if (value.getValue() instanceof long[]) {
      Logger.recordOutput(newKey, (long[]) value.getValue());
    } else if (value.getValue() instanceof long[][]) {
      Logger.recordOutput(newKey, (long[][]) value.getValue());
    } else if (value.getValue() instanceof Float) {
      if (value.getUnit() == null) {
        Logger.recordOutput(newKey, (Float) value.getValue());
      } else {
        Logger.recordOutput(newKey, (Float) value.getValue(), value.getUnit());
      }
    } else if (value.getValue() instanceof float[]) {
      Logger.recordOutput(newKey, (float[]) value.getValue());
    } else if (value.getValue() instanceof float[][]) {
      Logger.recordOutput(newKey, (float[][]) value.getValue());
    } else if (value.getValue() instanceof Double) {
      if (value.getUnit() == null) {
        Logger.recordOutput(newKey, (Double) value.getValue());
      } else {
        Logger.recordOutput(newKey, (Double) value.getValue(), value.getUnit());
      }
    } else if (value.getValue() instanceof DoubleSupplier) {
      Logger.recordOutput(newKey, (DoubleSupplier) value.getValue());
    } else if (value.getValue() instanceof double[]) {
      Logger.recordOutput(newKey, (double[]) value.getValue());
    } else if (value.getValue() instanceof double[][]) {
      Logger.recordOutput(newKey, (double[][]) value.getValue());
    } else if (value.getValue() instanceof String) {
      Logger.recordOutput(newKey, (String) value.getValue());
    } else if (value.getValue() instanceof String[]) {
      Logger.recordOutput(newKey, (String[]) value.getValue());
    } else if (value.getValue() instanceof String[][]) {
      Logger.recordOutput(newKey, (String[][]) value.getValue());
    } else if (value.getValue() instanceof Enum) {
      Logger.recordOutput(newKey, (Enum.class.cast(value.getValue())));
    } else if (value.getValue() instanceof Enum[]) {
      Logger.recordOutput(newKey, (Enum[]) value.getValue());
    } else if (value.getValue() instanceof Enum[][]) {
      Logger.recordOutput(newKey, (Enum[][]) value.getValue());
    } else if (value.getValue() instanceof Measure) {
      if (value.getUnit() == null) {
        Logger.recordOutput(newKey, (Measure<?>) value.getValue());
      } else {
        Logger.recordOutput(
            newKey, ((Measure<?>) value.getValue()).in(value.getUnit()), value.getUnit());
      }
    } else if (value.getValue() instanceof WPISerializable) {
      Logger.recordOutput(newKey, (WPISerializable) value.getValue());
    } else if (value.getValue() instanceof StructSerializable) {
      Logger.recordOutput(newKey, (StructSerializable) value.getValue());
    } else if (value.getValue() instanceof StructSerializable[]) {
      Logger.recordOutput(newKey, (StructSerializable[]) value.getValue());
    } else if (value.getValue() instanceof StructSerializable[][]) {
      Logger.recordOutput(newKey, (StructSerializable[][]) value.getValue());
    } else if (value.getValue() instanceof Record) {
      Logger.recordOutput(newKey, (Record) value.getValue());
    } else if (value.getValue() instanceof Record[]) {
      Logger.recordOutput(newKey, (Record[]) value.getValue());
    } else if (value.getValue() instanceof Record[][]) {
      Logger.recordOutput(newKey, (Record[][]) value.getValue());
    } else if (value.getValue() instanceof LoggedMechanism2d) {
      Logger.recordOutput(newKey, (LoggedMechanism2d) value.getValue());
    } else if (value.getValue() instanceof Color) {
      Logger.recordOutput(newKey, (Color) value.getValue());
    }
  }

  @SuppressWarnings("unchecked")
  private static void recordNullOutput(String newKey, StateValue value) {
    if (value.getValue() instanceof byte[]) {
      Logger.recordOutput(newKey, new byte[0]);
    } else if (value.getValue() instanceof byte[][]) {
      Logger.recordOutput(newKey, new byte[0][0]);
    } else if (value.getValue() instanceof Boolean) {
      Logger.recordOutput(newKey, false);
    } else if (value.getValue() instanceof BooleanSupplier) {
      Logger.recordOutput(newKey, (BooleanSupplier) () -> false);
    } else if (value.getValue() instanceof boolean[]) {
      Logger.recordOutput(newKey, new boolean[0]);
    } else if (value.getValue() instanceof boolean[][]) {
      Logger.recordOutput(newKey, new boolean[0][0]);
    } else if (value.getValue() instanceof Integer) {
      if (value.getUnit() == null) {
        Logger.recordOutput(newKey, Integer.MIN_VALUE);
      } else {
        Logger.recordOutput(newKey, Integer.MIN_VALUE, value.getUnit());
      }
    } else if (value.getValue() instanceof IntSupplier) {
      Logger.recordOutput(newKey, (IntSupplier) () -> Integer.MIN_VALUE);
    } else if (value.getValue() instanceof int[]) {
      Logger.recordOutput(newKey, new int[0]);
    } else if (value.getValue() instanceof int[][]) {
      Logger.recordOutput(newKey, new int[0][0]);
    } else if (value.getValue() instanceof Long) {
      if (value.getUnit() == null) {
        Logger.recordOutput(newKey, Long.MIN_VALUE);
      } else {
        Logger.recordOutput(newKey, Long.MIN_VALUE, value.getUnit());
      }
    } else if (value.getValue() instanceof LongSupplier) {
      Logger.recordOutput(newKey, (LongSupplier) () -> Long.MIN_VALUE);
    } else if (value.getValue() instanceof long[]) {
      Logger.recordOutput(newKey, new long[0]);
    } else if (value.getValue() instanceof long[][]) {
      Logger.recordOutput(newKey, new long[0][0]);
    } else if (value.getValue() instanceof Float) {
      if (value.getUnit() == null) {
        Logger.recordOutput(newKey, Float.NaN);
      } else {
        Logger.recordOutput(newKey, Float.NaN, value.getUnit());
      }
    } else if (value.getValue() instanceof float[]) {
      Logger.recordOutput(newKey, new float[0]);
    } else if (value.getValue() instanceof float[][]) {
      Logger.recordOutput(newKey, new float[0][0]);
    } else if (value.getValue() instanceof Double) {
      if (value.getUnit() == null) {
        Logger.recordOutput(newKey, Double.NaN);
      } else {
        Logger.recordOutput(newKey, Double.NaN, value.getUnit());
      }
    } else if (value.getValue() instanceof DoubleSupplier) {
      Logger.recordOutput(newKey, (DoubleSupplier) () -> Double.NaN);
    } else if (value.getValue() instanceof double[]) {
      Logger.recordOutput(newKey, new double[0]);
    } else if (value.getValue() instanceof double[][]) {
      Logger.recordOutput(newKey, new double[0][0]);
    } else if (value.getValue() instanceof String) {
      Logger.recordOutput(newKey, "");
    } else if (value.getValue() instanceof String[]) {
      Logger.recordOutput(newKey, new String[0]);
    } else if (value.getValue() instanceof String[][]) {
      Logger.recordOutput(newKey, new String[0][0]);
    } else if (value.getValue() instanceof Enum) {
      Logger.recordOutput(newKey, "");
    } else if (value.getValue() instanceof Enum[]) {
      Logger.recordOutput(newKey, new Enum[0]);
    } else if (value.getValue() instanceof Enum[][]) {
      Logger.recordOutput(newKey, new Enum[0][0]);
    } else if (value.getValue() instanceof Measure) {
      if (value.getUnit() == null) {
        Logger.recordOutput(newKey, ((Measure<?>) value.getValue()).unit().of(Double.NaN));
      } else {
        Logger.recordOutput(newKey, Double.NaN, value.getUnit());
      }
    } else if (value.getValue() instanceof WPISerializable) {
      Logger.recordOutput(newKey, (WPISerializable) value.getValue());
    } else if (value.getValue() instanceof StructSerializable) {
      Logger.recordOutput(newKey, (StructSerializable) value.getValue());
    } else if (value.getValue() instanceof StructSerializable[]) {
      Logger.recordOutput(newKey, new StructSerializable[0]);
    } else if (value.getValue() instanceof StructSerializable[][]) {
      Logger.recordOutput(newKey, new StructSerializable[0][0]);
    } else if (value.getValue() instanceof Record) {
      Logger.recordOutput(newKey, (Record) value.getValue());
    } else if (value.getValue() instanceof Record[]) {
      Logger.recordOutput(newKey, new Record[0]);
    } else if (value.getValue() instanceof Record[][]) {
      Logger.recordOutput(newKey, new Record[0][0]);
    } else if (value.getValue() instanceof LoggedMechanism2d) {
      Logger.recordOutput(newKey, new LoggedMechanism2d(0, 0));
    } else if (value.getValue() instanceof Color) {
      Logger.recordOutput(newKey, new Color(0, 0, 0));
    }
  }

  public static class StateValue {
    private final Object value;
    private final String name;
    private final Unit unit;

    private <U extends Unit> StateValue(Object value, String name, U unit) {
      this.value = value;
      this.name = name;
      this.unit = unit;
    }

    public Object getValue() {
      return value;
    }

    public String getName() {
      return name;
    }

    @SuppressWarnings("unchecked")
    public <U extends Unit> U getUnit() {
      return (U) unit;
    }

    public StateValue(byte[] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(byte[][] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(boolean value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(BooleanSupplier value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(boolean[] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(boolean[][] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(int value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(IntSupplier value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(int[] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(int[][] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(long value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(LongSupplier value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(long[] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(long[][] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(float value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(float[] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(float[][] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(double value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(DoubleSupplier value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(double[] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(double[][] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(String value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(String[] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(String[][] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public <E extends Enum<E>> StateValue(E value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public <E extends Enum<E>> StateValue(E[] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public <E extends Enum<E>> StateValue(E[][] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(Measure<?> value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(WPISerializable value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(StructSerializable value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(StructSerializable[] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(StructSerializable[][] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(Record value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(Record[] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(Record[][] value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(LoggedMechanism2d value, String name, Unit unit) {
      this((Object) value, name, unit);
    }

    public StateValue(Color value, String name, Unit unit) {
      this((Object) value, name, unit);
    }
  }

  public interface State {
    public abstract StateValue[] getValues();

    public default State create(StateValue... values) {
      return new State() {
        public StateValue[] getValues() {
          return values;
        }
      };
    }
  }

  public static class BaseState implements State {
    private final StateValue[] values;

    public BaseState(StateValue... values) {
      this.values = values;
    }

    public StateValue[] getValues() {
      return values;
    }
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
    public StateValue[] getValues() {
      return new StateValue[] {new StateValue(V, logName, Volts)};
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
    public StateValue[] getValues() {
      return new StateValue[] {new StateValue(A, logName, Amps)};
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
    public StateValue[] getValues() {
      return new StateValue[] {new StateValue(m, logName, Meters)};
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
    public StateValue[] getValues() {
      return new StateValue[] {
        new StateValue(m, logNames[0], Meters), new StateValue(mPs, logNames[1], MetersPerSecond)
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
    public StateValue[] getValues() {
      return new StateValue[] {new StateValue(rad, logName, Radians)};
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
    public StateValue[] getValues() {
      return new StateValue[] {new StateValue(radPs, logName, RadiansPerSecond)};
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
    public StateValue[] getValues() {
      return new StateValue[] {
        new StateValue(rad, logNames[0], Radians),
        new StateValue(radPs, logNames[1], RadiansPerSecond)
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

  public static class AngularVA_State implements Angular, VelAcc_State {
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
    public StateValue[] getValues() {
      return new StateValue[] {
        new StateValue(radPs, logNames[0], RadiansPerSecond),
        new StateValue(radPs2, logNames[1], RadiansPerSecondPerSecond)
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
