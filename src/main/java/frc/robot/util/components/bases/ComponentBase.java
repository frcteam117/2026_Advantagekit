package frc.robot.util.components.bases;

import frc.robot.util.StateUtil.State;

public interface ComponentBase {
  public abstract String[] getComponentNames();

  public abstract State[] getState();
}
