package frc.robot.util.control_functions;

import frc.robot.util.StateUtil.State;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

public abstract class ControlFunctionBase {
  protected List<Method> calculateMethods = new ArrayList<>(3);
  protected List<Method> updateMethods = new ArrayList<>(3);
  protected List<Method> resetMethods = new ArrayList<>(3);

  public ControlFunctionBase() {
    Method[] methods = this.getClass().getMethods();
    for (Method method : methods) {
      if (method.getName().equals("calculate")) {
        Class<?>[] paramTypes = method.getParameterTypes();
        if (paramTypes.length == 2
            && !ControlFunctionBase.class.equals(method.getDeclaringClass())
            && State.class.isAssignableFrom(paramTypes[0])
            && State.class.isAssignableFrom(paramTypes[1])) {
          int index = 0;
          // TODO Maybe make this take into account the second parameter
          for (int i = 0; i < calculateMethods.size(); i++) {
            if (paramTypes[0].isAssignableFrom(calculateMethods.get(i).getParameterTypes()[0])) {
              index = i + 1;
            }
          }
          calculateMethods.add(index, method);
        }
      } else if (method.getName().equals("updateState")) {
        Class<?>[] paramTypes = method.getParameterTypes();
        if (paramTypes.length == 1
            && !ControlFunctionBase.class.equals(method.getDeclaringClass())
            && State.class.isAssignableFrom(paramTypes[0])) {
          int index = 0;
          for (int i = 0; i < updateMethods.size(); i++) {
            if (paramTypes[0].isAssignableFrom(updateMethods.get(i).getParameterTypes()[0])) {
              index = i + 1;
            }
          }
          updateMethods.add(index, method);
        }
      } else if (method.getName().equals("resetState")) {
        Class<?>[] paramTypes = method.getParameterTypes();
        if (paramTypes.length == 1
            && !ControlFunctionBase.class.equals(method.getDeclaringClass())
            && State.class.isAssignableFrom(paramTypes[0])) {
          int index = 0;
          for (int i = 0; i < resetMethods.size(); i++) {
            if (paramTypes[0].isAssignableFrom(resetMethods.get(i).getParameterTypes()[0])) {
              index = i + 1;
            }
          }
          resetMethods.add(index, method);
        }
      }
    }
  }

  public final State calculate(Object goal_State, Object mechanism_State) {
    for (Method m : calculateMethods) {
      Class<?>[] paramTypes = m.getParameterTypes();
      if (paramTypes[0].isAssignableFrom(goal_State.getClass())
          && paramTypes[1].isAssignableFrom(mechanism_State.getClass())) {
        try {
          return (State) m.invoke(this, goal_State, mechanism_State);
        } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
    }
    // TODO Make this an exception
    return null;
  }

  public final void updateState(Object mechanism_State) {
    for (Method m : updateMethods) {
      if (m.getParameterTypes()[0].isAssignableFrom(mechanism_State.getClass())) {
        try {
          m.invoke(this, mechanism_State);
        } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
    }
    // TODO Make this an exception
  }

  public final void resetState(Object new_State) {
    for (Method m : resetMethods) {
      if (m.getParameterTypes()[0].isAssignableFrom(new_State.getClass())) {
        try {
          m.invoke(this, new_State);
        } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
    }
    // TODO Make this an exception
  }

  public abstract String getControlFunctionName();

  public abstract void setTunable(boolean isTunable);
}
