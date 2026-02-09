package frc.robot.util.components.bases;

import frc.robot.util.States.State;

public interface ComponentBase {
  public abstract String[] getComponentNames();

  public abstract State[] getState();
}
