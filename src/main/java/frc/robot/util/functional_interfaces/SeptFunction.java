package frc.robot.util.functional_interfaces;

@FunctionalInterface
public interface SeptFunction<T, U, V, W, X, Y, Z, R> {
  public R apply(T t, U u, V v, W w, X x, Y y, Z z);
}
