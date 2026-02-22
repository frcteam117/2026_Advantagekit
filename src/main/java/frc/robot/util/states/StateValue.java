package frc.robot.util.states;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.util.Color;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

public class StateValue {
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

  // byte[]

  public StateValue(byte[] value) {
    this(value, "", null);
  }

  public StateValue(byte[] value, String name) {
    this(value, name, null);
  }

  public StateValue(byte[] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(byte[] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // byte[][]

  public StateValue(byte[][] value) {
    this(value, "", null);
  }

  public StateValue(byte[][] value, String name) {
    this(value, name, null);
  }

  public StateValue(byte[][] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(byte[][] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // boolean

  public StateValue(boolean value) {
    this(value, "", null);
  }

  public StateValue(boolean value, String name) {
    this(value, name, null);
  }

  public StateValue(boolean value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(boolean value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // BooleanSupplier

  public StateValue(BooleanSupplier value) {
    this(value, "", null);
  }

  public StateValue(BooleanSupplier value, String name) {
    this(value, name, null);
  }

  public StateValue(BooleanSupplier value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(BooleanSupplier value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // boolean[]

  public StateValue(boolean[] value) {
    this(value, "", null);
  }

  public StateValue(boolean[] value, String name) {
    this(value, name, null);
  }

  public StateValue(boolean[] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(boolean[] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // boolean[][]

  public StateValue(boolean[][] value) {
    this(value, "", null);
  }

  public StateValue(boolean[][] value, String name) {
    this(value, name, null);
  }

  public StateValue(boolean[][] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(boolean[][] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // int

  public StateValue(int value) {
    this(value, "", null);
  }

  public StateValue(int value, String name) {
    this(value, name, null);
  }

  public StateValue(int value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(int value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // IntSupplier

  public StateValue(IntSupplier value) {
    this(value, "", null);
  }

  public StateValue(IntSupplier value, String name) {
    this(value, name, null);
  }

  public StateValue(IntSupplier value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(IntSupplier value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // int[]

  public StateValue(int[] value) {
    this(value, "", null);
  }

  public StateValue(int[] value, String name) {
    this(value, name, null);
  }

  public StateValue(int[] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(int[] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // int[][]

  public StateValue(int[][] value) {
    this(value, "", null);
  }

  public StateValue(int[][] value, String name) {
    this(value, name, null);
  }

  public StateValue(int[][] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(int[][] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // long

  public StateValue(long value) {
    this(value, "", null);
  }

  public StateValue(long value, String name) {
    this(value, name, null);
  }

  public StateValue(long value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(long value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // LongSupplier

  public StateValue(LongSupplier value) {
    this(value, "", null);
  }

  public StateValue(LongSupplier value, String name) {
    this(value, name, null);
  }

  public StateValue(LongSupplier value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(LongSupplier value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // long[]

  public StateValue(long[] value) {
    this(value, "", null);
  }

  public StateValue(long[] value, String name) {
    this(value, name, null);
  }

  public StateValue(long[] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(long[] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // long[][]

  public StateValue(long[][] value) {
    this(value, "", null);
  }

  public StateValue(long[][] value, String name) {
    this(value, name, null);
  }

  public StateValue(long[][] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(long[][] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // float

  public StateValue(float value) {
    this(value, "", null);
  }

  public StateValue(float value, String name) {
    this(value, name, null);
  }

  public StateValue(float value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(float value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // float[]

  public StateValue(float[] value) {
    this(value, "", null);
  }

  public StateValue(float[] value, String name) {
    this(value, name, null);
  }

  public StateValue(float[] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(float[] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // float[][]

  public StateValue(float[][] value) {
    this(value, "", null);
  }

  public StateValue(float[][] value, String name) {
    this(value, name, null);
  }

  public StateValue(float[][] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(float[][] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // double

  public StateValue(double value) {
    this(value, "", null);
  }

  public StateValue(double value, String name) {
    this(value, name, null);
  }

  public StateValue(double value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(double value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // DoubleSupplier

  public StateValue(DoubleSupplier value) {
    this(value, "", null);
  }

  public StateValue(DoubleSupplier value, String name) {
    this(value, name, null);
  }

  public StateValue(DoubleSupplier value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(DoubleSupplier value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // double[]

  public StateValue(double[] value) {
    this(value, "", null);
  }

  public StateValue(double[] value, String name) {
    this(value, name, null);
  }

  public StateValue(double[] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(double[] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // double[][]

  public StateValue(double[][] value) {
    this(value, "", null);
  }

  public StateValue(double[][] value, String name) {
    this(value, name, null);
  }

  public StateValue(double[][] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(double[][] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // String

  public StateValue(String value) {
    this(value, "", null);
  }

  public StateValue(String value, String name) {
    this(value, name, null);
  }

  public StateValue(String value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(String value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // String[]

  public StateValue(String[] value) {
    this(value, "", null);
  }

  public StateValue(String[] value, String name) {
    this(value, name, null);
  }

  public StateValue(String[] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(String[] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // String[][]

  public StateValue(String[][] value) {
    this(value, "", null);
  }

  public StateValue(String[][] value, String name) {
    this(value, name, null);
  }

  public StateValue(String[][] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(String[][] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // <E extends Enum<E>> E

  public <E extends Enum<E>> StateValue(E value) {
    this(value, "", null);
  }

  public <E extends Enum<E>> StateValue(E value, String name) {
    this(value, name, null);
  }

  public <E extends Enum<E>> StateValue(E value, Unit unit) {
    this(value, "", unit);
  }

  public <E extends Enum<E>> StateValue(E value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // <E extends Enum<E>> E[]

  public <E extends Enum<E>> StateValue(E[] value) {
    this(value, "", null);
  }

  public <E extends Enum<E>> StateValue(E[] value, String name) {
    this(value, name, null);
  }

  public <E extends Enum<E>> StateValue(E[] value, Unit unit) {
    this(value, "", unit);
  }

  public <E extends Enum<E>> StateValue(E[] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // <E extends Enum<E>> E[][]

  public <E extends Enum<E>> StateValue(E[][] value) {
    this(value, "", null);
  }

  public <E extends Enum<E>> StateValue(E[][] value, String name) {
    this(value, name, null);
  }

  public <E extends Enum<E>> StateValue(E[][] value, Unit unit) {
    this(value, "", unit);
  }

  public <E extends Enum<E>> StateValue(E[][] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // Measure<?>

  public StateValue(Measure<?> value) {
    this(value, "", null);
  }

  public StateValue(Measure<?> value, String name) {
    this(value, name, null);
  }

  public StateValue(Measure<?> value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(Measure<?> value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // WPISerializable

  public StateValue(WPISerializable value) {
    this(value, "", null);
  }

  public StateValue(WPISerializable value, String name) {
    this(value, name, null);
  }

  public StateValue(WPISerializable value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(WPISerializable value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // StructSerializable

  public StateValue(StructSerializable value) {
    this(value, "", null);
  }

  public StateValue(StructSerializable value, String name) {
    this(value, name, null);
  }

  public StateValue(StructSerializable value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(StructSerializable value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // StructSerializable[]

  public StateValue(StructSerializable[] value) {
    this(value, "", null);
  }

  public StateValue(StructSerializable[] value, String name) {
    this(value, name, null);
  }

  public StateValue(StructSerializable[] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(StructSerializable[] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // StructSerializable[][]

  public StateValue(StructSerializable[][] value) {
    this(value, "", null);
  }

  public StateValue(StructSerializable[][] value, String name) {
    this(value, name, null);
  }

  public StateValue(StructSerializable[][] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(StructSerializable[][] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // Record

  public StateValue(Record value) {
    this(value, "", null);
  }

  public StateValue(Record value, String name) {
    this(value, name, null);
  }

  public StateValue(Record value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(Record value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // Record[]

  public StateValue(Record[] value) {
    this(value, "", null);
  }

  public StateValue(Record[] value, String name) {
    this(value, name, null);
  }

  public StateValue(Record[] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(Record[] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // Record[][]

  public StateValue(Record[][] value) {
    this(value, "", null);
  }

  public StateValue(Record[][] value, String name) {
    this(value, name, null);
  }

  public StateValue(Record[][] value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(Record[][] value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // LoggedMechanism2d

  public StateValue(LoggedMechanism2d value) {
    this(value, "", null);
  }

  public StateValue(LoggedMechanism2d value, String name) {
    this(value, name, null);
  }

  public StateValue(LoggedMechanism2d value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(LoggedMechanism2d value, String name, Unit unit) {
    this((Object) value, name, unit);
  }

  // Color

  public StateValue(Color value) {
    this(value, "", null);
  }

  public StateValue(Color value, String name) {
    this(value, name, null);
  }

  public StateValue(Color value, Unit unit) {
    this(value, "", unit);
  }

  public StateValue(Color value, String name, Unit unit) {
    this((Object) value, name, unit);
  }
}
