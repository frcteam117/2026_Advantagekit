package frc.robot.util.states;

import edu.wpi.first.units.Measure;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.util.Color;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

public interface LoggableStateInputs extends LoggableInputs {
  public abstract State[] getStates();

  public abstract String[] getStateNames();

  public abstract void setStates(State[] states);

  @SuppressWarnings("unchecked")
  @Override
  public default void toLog(LogTable table) {
    State[] states = getStates();
    for (int i = 0; i < states.length; i++) {
      for (StateValue value : states[i].getValues()) {
        String newKey = getStateNames()[i] + "/" + StateUtil.toSuffix(value);
        if (value.getValue() instanceof byte[]) {
          table.put(newKey, (byte[]) value.getValue());
        } else if (value.getValue() instanceof byte[][]) {
          table.put(newKey, (byte[][]) value.getValue());
        } else if (value.getValue() instanceof Boolean) {
          table.put(newKey, (Boolean) value.getValue());
        } else if (value.getValue() instanceof BooleanSupplier) {
          table.put(newKey, ((BooleanSupplier) value.getValue()).getAsBoolean());
        } else if (value.getValue() instanceof boolean[]) {
          table.put(newKey, (boolean[]) value.getValue());
        } else if (value.getValue() instanceof boolean[][]) {
          table.put(newKey, (boolean[][]) value.getValue());
        } else if (value.getValue() instanceof Integer) {
          if (value.getUnit() == null) {
            table.put(newKey, (Integer) value.getValue());
          } else {
            table.put(newKey, (Integer) value.getValue(), value.getUnit().name());
          }
        } else if (value.getValue() instanceof IntSupplier) {
          table.put(newKey, ((IntSupplier) value.getValue()).getAsInt());
        } else if (value.getValue() instanceof int[]) {
          table.put(newKey, (int[]) value.getValue());
        } else if (value.getValue() instanceof int[][]) {
          table.put(newKey, (int[][]) value.getValue());
        } else if (value.getValue() instanceof Long) {
          if (value.getUnit() == null) {
            table.put(newKey, (Long) value.getValue());
          } else {
            table.put(newKey, (Long) value.getValue(), value.getUnit().name());
          }
        } else if (value.getValue() instanceof LongSupplier) {
          table.put(newKey, ((LongSupplier) value.getValue()).getAsLong());
        } else if (value.getValue() instanceof long[]) {
          table.put(newKey, (long[]) value.getValue());
        } else if (value.getValue() instanceof long[][]) {
          table.put(newKey, (long[][]) value.getValue());
        } else if (value.getValue() instanceof Float) {
          if (value.getUnit() == null) {
            table.put(newKey, (Float) value.getValue());
          } else {
            table.put(newKey, (Float) value.getValue(), value.getUnit().name());
          }
        } else if (value.getValue() instanceof float[]) {
          table.put(newKey, (float[]) value.getValue());
        } else if (value.getValue() instanceof float[][]) {
          table.put(newKey, (float[][]) value.getValue());
        } else if (value.getValue() instanceof Double) {
          if (value.getUnit() == null) {
            table.put(newKey, (Double) value.getValue());
          } else {
            table.put(newKey, (Double) value.getValue(), value.getUnit().name());
          }
        } else if (value.getValue() instanceof DoubleSupplier) {
          table.put(newKey, ((DoubleSupplier) value.getValue()).getAsDouble());
        } else if (value.getValue() instanceof double[]) {
          table.put(newKey, (double[]) value.getValue());
        } else if (value.getValue() instanceof double[][]) {
          table.put(newKey, (double[][]) value.getValue());
        } else if (value.getValue() instanceof String) {
          table.put(newKey, (String) value.getValue());
        } else if (value.getValue() instanceof String[]) {
          table.put(newKey, (String[]) value.getValue());
        } else if (value.getValue() instanceof String[][]) {
          table.put(newKey, (String[][]) value.getValue());
        } else if (value.getValue() instanceof Enum) {
          table.put(newKey, (Enum.class.cast(value.getValue())));
        } else if (value.getValue() instanceof Enum[]) {
          table.put(newKey, (Enum[]) value.getValue());
        } else if (value.getValue() instanceof Enum[][]) {
          table.put(newKey, (Enum[][]) value.getValue());
        } else if (value.getValue() instanceof Measure) {
          if (value.getUnit() == null) {
            table.put(newKey, (Measure<?>) value.getValue());
          } else {
            table.put(
                newKey,
                ((Measure<?>) value.getValue()).in(value.getUnit()),
                value.getUnit().name());
          }
        } else if (value.getValue() instanceof WPISerializable) {
          table.put(newKey, (WPISerializable) value.getValue());
        } else if (value.getValue() instanceof StructSerializable) {
          table.put(newKey, (StructSerializable) value.getValue());
        } else if (value.getValue() instanceof StructSerializable[]) {
          table.put(newKey, (StructSerializable[]) value.getValue());
        } else if (value.getValue() instanceof StructSerializable[][]) {
          table.put(newKey, (StructSerializable[][]) value.getValue());
        } else if (value.getValue() instanceof Record) {
          table.put(newKey, (Record) value.getValue());
        } else if (value.getValue() instanceof Record[]) {
          table.put(newKey, (Record[]) value.getValue());
        } else if (value.getValue() instanceof Record[][]) {
          table.put(newKey, (Record[][]) value.getValue());
        } else if (value.getValue() instanceof LoggedMechanism2d) {
          return;
        } else if (value.getValue() instanceof Color) {
          table.put(newKey, (Color) value.getValue());
        }
      }
    }
  }

  @SuppressWarnings("unchecked")
  @Override
  public default void fromLog(LogTable table) {
    State[] states = getStates();
    for (int i = 0; i < states.length; i++) {
      StateValue[] values = states[i].getValues();
      for (int j = 0; j < values.length; j++) {
        String newKey = getStateNames()[i] + "/" + StateUtil.toSuffix(values[j]);
        if (values[j].getValue() instanceof byte[]) {
          values[j] = new StateValue(
              table.get(newKey, (byte[]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof byte[][]) {
          values[j] = new StateValue(
              table.get(newKey, (byte[][]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof Boolean) {
          values[j] = new StateValue(
              table.get(newKey, (Boolean) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof BooleanSupplier) {
          boolean value =
              table.get(newKey, ((BooleanSupplier) values[j].getValue()).getAsBoolean());
          values[j] = new StateValue(
              (BooleanSupplier) () -> value, values[j].getName(), values[j].getUnit());
        } else if (values[j].getValue() instanceof boolean[]) {
          values[j] = new StateValue(
              table.get(newKey, (boolean[]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof boolean[][]) {
          values[j] = new StateValue(
              table.get(newKey, (boolean[][]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof Integer) {
          values[j] = new StateValue(
              table.get(newKey, (Integer) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof IntSupplier) {
          int value = table.get(newKey, ((IntSupplier) values[j].getValue()).getAsInt());
          values[j] =
              new StateValue((IntSupplier) () -> value, values[j].getName(), values[j].getUnit());
        } else if (values[j].getValue() instanceof int[]) {
          values[j] = new StateValue(
              table.get(newKey, (int[]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof int[][]) {
          values[j] = new StateValue(
              table.get(newKey, (int[][]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof Long) {
          values[j] = new StateValue(
              table.get(newKey, (Long) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof LongSupplier) {
          long value = table.get(newKey, ((LongSupplier) values[j].getValue()).getAsLong());
          values[j] =
              new StateValue((LongSupplier) () -> value, values[j].getName(), values[j].getUnit());
        } else if (values[j].getValue() instanceof long[]) {
          values[j] = new StateValue(
              table.get(newKey, (long[]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof long[][]) {
          values[j] = new StateValue(
              table.get(newKey, (long[][]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof Float) {
          values[j] = new StateValue(
              table.get(newKey, (Float) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof float[]) {
          values[j] = new StateValue(
              table.get(newKey, (float[]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof float[][]) {
          values[j] = new StateValue(
              table.get(newKey, (float[][]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof Double) {
          values[j] = new StateValue(
              table.get(newKey, (Double) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof DoubleSupplier) {
          double value = table.get(newKey, ((DoubleSupplier) values[j].getValue()).getAsDouble());
          values[j] = new StateValue(
              (DoubleSupplier) () -> value, values[j].getName(), values[j].getUnit());
        } else if (values[j].getValue() instanceof double[]) {
          values[j] = new StateValue(
              table.get(newKey, (double[]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof double[][]) {
          values[j] = new StateValue(
              table.get(newKey, (double[][]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof String) {
          values[j] = new StateValue(
              table.get(newKey, (String) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof String[]) {
          values[j] = new StateValue(
              table.get(newKey, (String[]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof String[][]) {
          values[j] = new StateValue(
              table.get(newKey, (String[][]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof Enum) {
          values[j] = new StateValue(
              table.get(newKey, (Enum.class.cast(values[j].getValue()))),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof Enum[]) {
          values[j] = new StateValue(
              table.get(newKey, (Enum[]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof Enum[][]) {
          values[j] = new StateValue(
              table.get(newKey, (Enum[][]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof Measure) {
          if (values[j].getUnit() == null) {
            values[j] = new StateValue(
                table.get(newKey, (Measure<?>) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else {
            values[j] = new StateValue(
                table.get(newKey, ((Measure<?>) values[j].getValue()).in(values[j].getUnit())),
                values[j].getName(),
                values[j].getUnit());
          }
        } else if (values[j].getValue() instanceof WPISerializable) {
          values[j] = new StateValue(
              table.get(newKey, (WPISerializable) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof StructSerializable) {
          values[j] = new StateValue(
              table.get(newKey, (StructSerializable) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof StructSerializable[]) {
          values[j] = new StateValue(
              table.get(newKey, (StructSerializable[]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof StructSerializable[][]) {
          values[j] = new StateValue(
              table.get(newKey, (StructSerializable[][]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof Record) {
          values[j] = new StateValue(
              table.get(newKey, (Record) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof Record[]) {
          values[j] = new StateValue(
              table.get(newKey, (Record[]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof Record[][]) {
          values[j] = new StateValue(
              table.get(newKey, (Record[][]) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        } else if (values[j].getValue() instanceof LoggedMechanism2d) {
          return;
        } else if (values[j].getValue() instanceof Color) {
          values[j] = new StateValue(
              table.get(newKey, (Color) values[j].getValue()),
              values[j].getName(),
              values[j].getUnit());
        }
      }
      states[i] = states[i].createNew(values);
    }
    setStates(states);
  }
}
