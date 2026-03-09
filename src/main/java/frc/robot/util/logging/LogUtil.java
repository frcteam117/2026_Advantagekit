package frc.robot.util.logging;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

  public static void createTunableProfiledPID(
      String key, ProfiledPIDController profiledPID, BooleanSupplier shouldPublish) {
    new TunableDouble(key + "/0 P", profiledPID.getP(), shouldPublish, profiledPID::setP); // 0 P
    new TunableDouble(key + "/1 I", profiledPID.getI(), shouldPublish, profiledPID::setI); // 1 I
    new TunableDouble(key + "/2 D", profiledPID.getD(), shouldPublish, profiledPID::setD); // 2 D
    new TunableDouble(
        key + "/3 IZone", profiledPID.getIZone(), shouldPublish, profiledPID::setIZone); // 3 IZone
    TunableDouble minIntegral =
        new TunableDouble(key + "/4 MinIntegral", -1, shouldPublish); // 4 min integral
    TunableDouble maxIntegral = new TunableDouble(
        key + "/5 MaxIntegral",
        1,
        shouldPublish,
        max -> profiledPID.setIntegratorRange(minIntegral.getAsDouble(), max)); // 5 max integral
    minIntegral.runOnChange(min -> profiledPID.setIntegratorRange(min, maxIntegral.getAsDouble()));
    TunableDouble maxV = new TunableDouble(
        key + "/6 MaxVelocity", profiledPID.getConstraints().maxVelocity, shouldPublish); // 6 max V
    TunableDouble maxA = new TunableDouble(
        key + "/7 MaxAcceleration",
        profiledPID.getConstraints().maxAcceleration,
        shouldPublish,
        value -> profiledPID.setConstraints(new Constraints(maxV.getAsDouble(), value))); // 7 max A
    minIntegral.runOnChange(
        value -> profiledPID.setConstraints(new Constraints(value, maxA.getAsDouble())));
    TunableDouble posTolerance = new TunableDouble(
        key + "/8 PosTolerance",
        profiledPID.getPositionTolerance(),
        shouldPublish); // 8 pos tolerance
    TunableDouble velTolerance = new TunableDouble(
        key + "/9 VelTolerance",
        profiledPID.getVelocityTolerance(),
        shouldPublish,
        value -> profiledPID.setTolerance(posTolerance.getAsDouble(), value)); // 9 vel tolerance
    posTolerance.runOnChange(value -> profiledPID.setTolerance(value, velTolerance.getAsDouble()));
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
