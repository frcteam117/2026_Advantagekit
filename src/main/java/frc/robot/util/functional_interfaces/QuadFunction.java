package frc.robot.util.functional_interfaces;

@FunctionalInterface
public interface QuadFunction<T, U, V, W, R> {
  public R apply(T t, U u, V v, W w);
}
