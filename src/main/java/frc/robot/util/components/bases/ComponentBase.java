package frc.robot.util.components.bases;

import frc.robot.util.components.bases.ComponentStates.ComponentState;

public interface ComponentBase {
  public abstract String[] getComponentNames();

  public abstract ComponentState[] getState();
}
