package frc.robot.util.components.bases;

import frc.robot.util.States.State;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

public abstract class ComponentControllerBase implements ComponentBase {
  protected List<Method> setInputMethods = new ArrayList<>(1);
  protected List<Method> resetMethods = new ArrayList<>(1);

  public ComponentControllerBase() {
    Method[] methods = this.getClass().getMethods();
    for (Method method : methods) {
      if (method.getName().equals("setInput")) {
        Class<?>[] paramTypes = method.getParameterTypes();
        if (paramTypes.length == 1
            && !ComponentControllerBase.class.equals(method.getDeclaringClass())
            && State.class.isAssignableFrom(paramTypes[0])) {
          int index = 0;
          for (int i = 0; i < setInputMethods.size(); i++) {
            if (paramTypes[0].isAssignableFrom(setInputMethods.get(i).getParameterTypes()[0])) {
              index = i + 1;
            }
          }
          setInputMethods.add(index, method);
        }
      } else if (method.getName().equals("resetState")) {
        Class<?>[] paramTypes = method.getParameterTypes();
        if (paramTypes.length == 1
            && !ComponentControllerBase.class.equals(method.getDeclaringClass())
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

  public final void setInput(State input_State) {
    for (Method m : setInputMethods) {
      if (m.getParameterTypes()[0].isAssignableFrom(input_State.getClass())) {
        try {
          m.invoke(this, input_State);
        } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
    }
    // TODO Make this an exception
  }

  public final void resetState(State new_State) {
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

  public abstract String getControllerName();
}
