package frc.robot.util.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class TunableString implements Supplier<String> {
  private final String key;
  private final BooleanSupplier shouldPublish;
  private Consumer<String> onChange;
  private String value;
  private LoggedNetworkString networkString = null;

  public TunableString(
      String key, String defaultValue, BooleanSupplier shouldPublish, Consumer<String> onChange) {
    this.key = key;
    this.shouldPublish = shouldPublish;
    this.onChange = onChange;
    value = defaultValue;
    LogUtil.getInstance().registerUpdateMethod(this::update);
  }

  public TunableString(String key, String defaultValue, BooleanSupplier shouldPublish) {
    this(key, defaultValue, shouldPublish, (value) -> {});
  }

  public void update() {
    if (shouldPublish.getAsBoolean()) {
      if (networkString == null) {
        networkString = new LoggedNetworkString(key, value);
      }
      if (value != networkString.get()) {
        value = networkString.get();
        onChange.accept(value);
      }
    } else {
      if (networkString != null) {
        networkString = null;
        NetworkTableInstance.getDefault().getStringTopic(key).getEntry(value).unpublish();
      }
    }
  }

  public void runOnChange(Consumer<String> onChange) {
    this.onChange = onChange;
  }

  @Override
  public String get() {
    return value;
  }
}
