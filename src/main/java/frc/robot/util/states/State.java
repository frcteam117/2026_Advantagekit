package frc.robot.util.states;

public interface State {
  public abstract StateValue[] getValues();

  public abstract State createNew(StateValue... values);

  public static State create(StateValue... values) {
    return new State() {
      public StateValue[] getValues() {
        return values;
      }

      public State createNew(StateValue... values) {
        return State.create(values);
      }
    };
  }
}
