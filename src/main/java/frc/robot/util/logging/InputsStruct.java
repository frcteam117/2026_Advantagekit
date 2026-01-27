package frc.robot.util.logging;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Function;

@SuppressWarnings("rawtypes")
public class InputsStruct<T> implements Struct<T> {
  private final Class<T> inputsClass;
  private Constructor<T> inputsConstructor = null;
  private final String typeName;
  private final int size;
  private final String schema;
  private final Struct<?>[] nestedStructs;

  private final List<BiConsumer<ByteBuffer, Object>> packFunctions = new ArrayList<>();
  private final List<Function<ByteBuffer, Object>> unpackFunctions = new ArrayList<>();

  public InputsStruct(Class<T> inputsClass, Unit... units) {
    this(inputsClass, inputsClass.getSimpleName(), units);
  }

  @SuppressWarnings("unchecked")
  public InputsStruct(Class<T> inputsClass, String typeName, Unit... units) {
    this.inputsClass = inputsClass;
    this.typeName = typeName;
    int num = 0;

    int size = 0;
    var schema = new StringBuilder();
    Field[] fields = inputsClass.getFields();
    List<Struct<?>> nestedStructs = new LinkedList<>();
    Class<?>[] parameterTypes;
    if (StructSerializable.class.isAssignableFrom(inputsClass)) {
      parameterTypes = new Class[fields.length - 1];
    } else {
      parameterTypes = new Class[fields.length];
    }
    for (int i = 0; i < fields.length; i++) {
      Field field = fields[i];

      // Add to parameter types
      if (!field.getName().equals("struct")) {
        parameterTypes[i] = field.getType();

        // Add functions based on type
        if (field.getType().equals(boolean.class)) {
          size += 1;
          schema.append("bool ");
          schema.append(field.getName());
          schema.append(";");
          packFunctions.add((ByteBuffer bb, Object inputs) -> {
            try {
              bb.put(((boolean) field.get(inputs)) ? (byte) 1 : (byte) 0);
            } catch (IllegalAccessException | IllegalArgumentException e) {
              e.printStackTrace();
            }
          });
          unpackFunctions.add((ByteBuffer bb) -> bb.get() != 0);

        } else if (field.getType().equals(short.class)) {
          size += 2;
          schema.append("int16 ");
          schema.append(field.getName());
          schema.append(";");
          packFunctions.add((ByteBuffer bb, Object inputs) -> {
            try {
              bb.putShort((short) field.get(inputs));
            } catch (IllegalAccessException | IllegalArgumentException e) {
              e.printStackTrace();
            }
          });
          unpackFunctions.add((ByteBuffer bb) -> bb.getShort());

        } else if (field.getType().equals(int.class)) {
          size += 4;
          schema.append("int32 ");
          schema.append(field.getName());
          schema.append(";");
          packFunctions.add((ByteBuffer bb, Object inputs) -> {
            try {
              bb.putInt((int) field.get(inputs));
            } catch (IllegalAccessException | IllegalArgumentException e) {
              e.printStackTrace();
            }
          });
          unpackFunctions.add((ByteBuffer bb) -> bb.getInt());

        } else if (field.getType().equals(long.class)) {
          size += 8;
          schema.append("int64 ");
          schema.append(field.getName());
          schema.append(";");
          packFunctions.add((ByteBuffer bb, Object inputs) -> {
            try {
              bb.putLong((long) field.get(inputs));
            } catch (IllegalAccessException | IllegalArgumentException e) {
              e.printStackTrace();
            }
          });
          unpackFunctions.add((ByteBuffer bb) -> bb.getLong());

        } else if (field.getType().equals(float.class)) {
          size += 4;
          schema.append("float ");
          schema.append(field.getName());
          schema.append(";");
          packFunctions.add((ByteBuffer bb, Object inputs) -> {
            try {
              bb.putFloat((float) field.get(inputs));
            } catch (IllegalAccessException | IllegalArgumentException e) {
              e.printStackTrace();
            }
          });
          unpackFunctions.add((ByteBuffer bb) -> bb.getFloat());

        } else if (field.getType().equals(double.class)) {
          size += 8;
          schema.append("double ");
          schema.append(field.getName());
          schema.append(";");
          packFunctions.add((ByteBuffer bb, Object inputs) -> {
            try {
              bb.putDouble((double) field.get(inputs));
            } catch (IllegalAccessException | IllegalArgumentException e) {
              e.printStackTrace();
            }
          });
          unpackFunctions.add((ByteBuffer bb) -> bb.getDouble());

        } else if (Measure.class.isAssignableFrom(field.getType())) {
          // Unit tempUnit = null;
          // if (units.length > num) {
          //   tempUnit = units[num];
          // } else {
          //   try {
          //     tempUnit =
          //         (Unit)
          //             ((Class<Measure>) component.getType())
          //                 .getDeclaredMethod("baseUnit")
          //                 .invoke(component.getAccessor().invoke());
          //   } catch (IllegalAccessException
          //       | IllegalArgumentException
          //       | InvocationTargetException
          //       | NoSuchMethodException
          //       | SecurityException
          //       | InstantiationException e) {
          //     e.printStackTrace();
          //   }
          // }
          // TODO Make this work and add warning for if tempUnit is still null
          final Unit unit = units[num];
          size += 8;
          schema.append("double ");
          schema.append(field.getName() + LogUtil.toSuffix(unit.symbol()));
          num++;
          schema.append(";");
          packFunctions.add((ByteBuffer bb, Object inputs) -> {
            try {
              bb.putDouble((double) ((Measure) field.get(inputs)).in(unit));
            } catch (IllegalAccessException | IllegalArgumentException e) {
              e.printStackTrace();
            }
          });
          unpackFunctions.add((ByteBuffer bb) -> unit.of(bb.getDouble()));

        } else if (field.getType().isEnum()) {
          size += 4;
          Enum[] enumValues = (Enum[]) field.getType().getEnumConstants();
          schema.append("enum {");
          for (int j = 0; j < enumValues.length; j++) {
            schema.append(enumValues[j].name());
            schema.append("=");
            schema.append(enumValues[j].ordinal());
            if (j < enumValues.length - 1) {
              schema.append(", ");
            }
          }
          schema.append("} int32 ");
          schema.append(field.getName());
          schema.append(";");
          packFunctions.add((ByteBuffer bb, Object inputs) -> {
            try {
              bb.putInt(((Enum) field.get(inputs)).ordinal());
            } catch (IllegalAccessException | IllegalArgumentException e) {
              e.printStackTrace();
            }
          });
          unpackFunctions.add((ByteBuffer bb) -> enumValues[bb.getInt()]);

        } else if (field.getType().isRecord()
            || StructSerializable.class.isAssignableFrom(field.getType())) {
          Struct<?> struct = null;
          if (field.getType().isRecord()) {
            struct = new RecordStruct(field.getType());
          } else {
            try {
              struct = (Struct) field.getType().getDeclaredField("struct").get(null);
            } catch (IllegalArgumentException
                | IllegalAccessException
                | NoSuchFieldException
                | SecurityException e) {
              e.printStackTrace();
            }
          }

          if (struct == null) {
            DriverStation.reportError(
                "[AdvantageKit] Failed to load nested struct \""
                    + field.getName()
                    + "\" for inputs type \""
                    + inputsClass.getSimpleName()
                    + "\"",
                true);
            packFunctions.add((ByteBuffer bb, Object inputs) -> {});
            unpackFunctions.add((ByteBuffer bb) -> null);
          } else {
            size += struct.getSize();
            schema.append(struct.getTypeName());
            schema.append(" ");
            schema.append(field.getName());
            schema.append(";");
            nestedStructs.add(struct);
            Struct rawStruct = struct;
            packFunctions.add((ByteBuffer bb, Object inputs) -> {
              try {
                rawStruct.pack(bb, field.get(inputs));
              } catch (IllegalAccessException | IllegalArgumentException e) {
                e.printStackTrace();
              }
            });
            unpackFunctions.add((ByteBuffer bb) -> rawStruct.unpack(bb));
          }

        } else {
          DriverStation.reportError(
              "[AdvantageKit] Field \""
                  + field.getName()
                  + "\" for inputs type \""
                  + inputsClass.getSimpleName()
                  + "\" uses an unsupported type and will not be logged. Check the implementation.",
              true);
          packFunctions.add((ByteBuffer bb, Object inputs) -> {});
          unpackFunctions.add((ByteBuffer bb) -> null);
        }
      }
    }

    // Save schema
    this.size = size;
    this.schema = schema.toString();

    // Save nested structs
    this.nestedStructs = new Struct[nestedStructs.size()];
    for (int i = 0; i < nestedStructs.size(); i++) {
      this.nestedStructs[i] = nestedStructs.get(i);
    }

    // Get constructor
    try {
      this.inputsConstructor = inputsClass.getDeclaredConstructor();
      this.inputsConstructor.setAccessible(true);
    } catch (NoSuchMethodException | SecurityException e) {
      e.printStackTrace();
    }
  }

  @Override
  public Class<T> getTypeClass() {
    return inputsClass;
  }

  @Override
  public String getTypeName() {
    return typeName;
  }

  @Override
  public int getSize() {
    return size;
  }

  @Override
  public String getSchema() {
    return schema;
  }

  @Override
  public Struct<?>[] getNested() {
    return nestedStructs;
  }

  @Override
  public T unpack(ByteBuffer bb) {
    // Exit if no constructor available
    if (inputsConstructor == null) {
      return null;
    }

    // Construct inputs
    T output = null;
    try {
      output = inputsConstructor.newInstance();
    } catch (InstantiationException
        | IllegalAccessException
        | IllegalArgumentException
        | InvocationTargetException e) {
      e.printStackTrace();
    }

    // Unpack elements
    Field[] fields = inputsClass.getDeclaredFields();
    for (int i = 0; i < unpackFunctions.size(); i++) {
      fields[i].setAccessible(true);
      try {
        fields[i].set(output, unpackFunctions.get(i).apply(bb));
      } catch (IllegalArgumentException | IllegalAccessException e) {
        e.printStackTrace();
      }
    }

    return output;
  }

  @Override
  public void pack(ByteBuffer bb, Object value) {
    for (var function : packFunctions) {
      function.accept(bb, value);
    }
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
