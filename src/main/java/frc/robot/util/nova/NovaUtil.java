package frc.robot.util.nova;

import static edu.wpi.first.units.Units.Seconds;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.Error;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.SimulatedArena;

public class NovaUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(
      ThriftyNova nova,
      int maxAttempts,
      Runnable command,
      BooleanSupplier isCorrect,
      String onSuccess,
      String onFailure,
      Error... errors) {
    for (int i = 0; i < maxAttempts; i++) {
      // boolean containedError = false;
      command.run();
      if (isCorrect.getAsBoolean()) {
        System.out.println(onSuccess + nova.getID());
        break;
      } else {
        System.out.println(onFailure + nova.getID());
        try {
          Thread.sleep(20);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
        // for (Error error : errors) {
        //     if (nova.errors.contains(error)) {
        //         containedError = true;
        //     }
        //     nova.errors.remove(error);
        // }

        // if (containedError == false) {
        //     break;
        // }
      }
    }
  }

  public static double[] getSimulationOdometryTimeStamps() {
    final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] =
          Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
    }

    return odometryTimeStamps;
  }
}
