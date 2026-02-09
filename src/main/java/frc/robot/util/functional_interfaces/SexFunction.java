package frc.robot.util.functional_interfaces;

@FunctionalInterface
public interface SexFunction<T, U, V, W, X, Y, R> {
  public R apply(T t, U u, V v, W w, X x, Y y);
}
