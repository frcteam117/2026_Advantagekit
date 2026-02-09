package frc.robot.util.functional_interfaces;

@FunctionalInterface
public interface QuinFunction<T, U, V, W, X, R> {
  public R apply(T t, U u, V v, W w, X x);
}
