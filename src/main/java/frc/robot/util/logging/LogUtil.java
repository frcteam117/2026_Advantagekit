package frc.robot.util.logging;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class LogUtil {
  private static LogUtil instance = null;
  private final List<Runnable> updateMethods = new ArrayList<>();

  public static LogUtil getInstance() {
    if (instance == null) {
      instance = new LogUtil();
    }
    return instance;
  }

  public void registerUpdateMethod(Runnable updateMethod) {
    updateMethods.add(updateMethod);
  }

  public void runUpdateMethods() {
    for (Runnable runnable : updateMethods) {
      runnable.run();
    }
  }

  public static String toSuffix(String symbol) {
    return "_"
        + symbol
            .replace("u", "µ")
            .replace("*", "·")
            .replace('K', 'k')
            .replace("/", " ̸ ")
            .replace("<?>", "value");
  }

  public static void createTunablePID(
      String key, PIDController pidController, BooleanSupplier shouldPublish) {
    @SuppressWarnings("unused")
    TunableDouble
        P = new TunableDouble(key + "P", pidController.getP(), shouldPublish, pidController::setP),
        I = new TunableDouble(key + "I", pidController.getI(), shouldPublish, pidController::setI),
        D = new TunableDouble(key + "D", pidController.getD(), shouldPublish, pidController::setD);
  }

  public static void createTunableFF(
      String key, SimpleMotorFeedforward ff, BooleanSupplier shouldPublish) {
    @SuppressWarnings("unused")
    TunableDouble S = new TunableDouble(key + "S", ff.getKs(), shouldPublish, ff::setKs),
        V = new TunableDouble(key + "V", ff.getKv(), shouldPublish, ff::setKv),
        A = new TunableDouble(key + "A", ff.getKa(), shouldPublish, ff::setKa);
  }

  public static void createTunableFF(String key, ArmFeedforward ff, BooleanSupplier shouldPublish) {
    @SuppressWarnings("unused")
    TunableDouble S = new TunableDouble(key + "S", ff.getKs(), shouldPublish, ff::setKs),
        G = new TunableDouble(key + "G", ff.getKg(), shouldPublish, ff::setKg),
        V = new TunableDouble(key + "V", ff.getKv(), shouldPublish, ff::setKv),
        A = new TunableDouble(key + "A", ff.getKa(), shouldPublish, ff::setKa);
  }

  public record AngularMechanismState(
      double mechanism_rad,
      double mechanism_radPs,
      double stator_V,
      double stator_A,
      double supply_A) {}

  public record LinearMechanismState(
      double mechanism_m,
      double mechanism_mPs,
      double stator_V,
      double stator_A,
      double supply_A) {}

  public record AngularSetpoint(double V, double rad, double radPs) {}

  public record LinearSetpoint(double V, double m, double mPs) {}
}
