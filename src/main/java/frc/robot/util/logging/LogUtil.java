package frc.robot.util.logging;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
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
            .replace("/", "Per") // " ̸ ")
            .replace("<?>", "value");
  }

  public static void createTunablePID(
      String key, PIDController pidController, BooleanSupplier shouldPublish) {
    new TunableDouble(key + "/0 P", pidController.getP(), shouldPublish, pidController::setP);
    new TunableDouble(key + "/1 I", pidController.getI(), shouldPublish, pidController::setI);
    new TunableDouble(key + "/2 D", pidController.getD(), shouldPublish, pidController::setD);
    new TunableDouble(
        key + "/3 IZone", pidController.getIZone(), shouldPublish, pidController::setIZone);
    TunableDouble minIntegral = new TunableDouble(key + "/4 MinIntegral", -1, shouldPublish);
    TunableDouble maxIntegral = new TunableDouble(
        key + "/5 MaxIntegral",
        1,
        shouldPublish,
        max -> pidController.setIntegratorRange(minIntegral.getAsDouble(), max));
    minIntegral.runOnChange(
        min -> pidController.setIntegratorRange(min, maxIntegral.getAsDouble()));
  }

  // public static void createTunablePID(
  //     String key, SparkBase spark, BooleanSupplier shouldPublish) {
  //   new TunableDouble(key + "/0 P", spark.configure, shouldPublish, pidController::setP);
  //   new TunableDouble(key + "/1 I", pidController.getI(), shouldPublish, pidController::setI);
  //   new TunableDouble(key + "/2 D", pidController.getD(), shouldPublish, pidController::setD);
  //   new TunableDouble(
  //       key + "/3 IZone", pidController.getIZone(), shouldPublish, pidController::setIZone);
  //   TunableDouble minIntegral = new TunableDouble(key + "/4 MinIntegral", -1, shouldPublish);
  //   TunableDouble maxIntegral = new TunableDouble(
  //       key + "/5 MaxIntegral",
  //       1,
  //       shouldPublish,
  //       max -> pidController.setIntegratorRange(minIntegral.getAsDouble(), max));
  //   minIntegral.runOnChange(
  //       min -> pidController.setIntegratorRange(min, maxIntegral.getAsDouble()));
  // }

  public static void createTunableFF(
      String key, SimpleMotorFeedforward ff, BooleanSupplier shouldPublish) {
    new TunableDouble(key + "/6 S", ff.getKs(), shouldPublish, ff::setKs);
    new TunableDouble(key + "/7 V", ff.getKv(), shouldPublish, ff::setKv);
    new TunableDouble(key + "/8 A", ff.getKa(), shouldPublish, ff::setKa);
  }

  public static void createTunableFF(String key, ArmFeedforward ff, BooleanSupplier shouldPublish) {
    new TunableDouble(key + "/6 S", ff.getKs(), shouldPublish, ff::setKs);
    new TunableDouble(key + "/7 V", ff.getKv(), shouldPublish, ff::setKv);
    new TunableDouble(key + "/8 A", ff.getKa(), shouldPublish, ff::setKa);
    new TunableDouble(key + "/9 G", ff.getKg(), shouldPublish, ff::setKg);
  }

  public static void createTunableFF(
      String key, ElevatorFeedforward ff, BooleanSupplier shouldPublish) {
    new TunableDouble(key + "/6 S", ff.getKs(), shouldPublish, ff::setKs);
    new TunableDouble(key + "/7 V", ff.getKv(), shouldPublish, ff::setKv);
    new TunableDouble(key + "/8 A", ff.getKa(), shouldPublish, ff::setKa);
    new TunableDouble(key + "/9 G", ff.getKg(), shouldPublish, ff::setKg);
  }
}
