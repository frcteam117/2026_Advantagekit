package frc.robot.util.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.function.BooleanConsumer;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class TunableBoolean implements BooleanSupplier {
  private final String key;
  private final BooleanSupplier shouldPublish;
  private final BooleanConsumer onChange;
  private boolean value;
  private LoggedNetworkBoolean networkBoolean = null;

  public TunableBoolean(
      String key, boolean defaultValue, BooleanSupplier shouldPublish, BooleanConsumer onChange) {
    this.key = key;
    this.shouldPublish = shouldPublish;
    this.onChange = onChange;
    value = defaultValue;
    LogUtil.getInstance().registerUpdateMethod(this::update);
  }

  public TunableBoolean(String key, boolean defaultValue, BooleanSupplier shouldPublish) {
    this(key, defaultValue, shouldPublish, (value) -> {});
  }

  public void update() {
    if (shouldPublish.getAsBoolean()) {
      if (networkBoolean == null) {
        networkBoolean = new LoggedNetworkBoolean(key, value);
      }
      if (value != networkBoolean.get()) {
        value = networkBoolean.get();
        onChange.accept(value);
      }
    } else {
      if (networkBoolean != null) {
        networkBoolean = null;
        NetworkTableInstance.getDefault().getBooleanTopic(key).getEntry(value).unpublish();
      }
    }
  }

  @Override
  public boolean getAsBoolean() {
    return value;
  }
}
