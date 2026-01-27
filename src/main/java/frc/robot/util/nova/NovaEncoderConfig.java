package frc.robot.util.nova;

import com.thethriftybot.devices.ThriftyNova.EncoderType;
import com.thethriftybot.devices.ThriftyNova.ExternalEncoder;

public class NovaEncoderConfig extends NovaBaseConfig {
  /**
   * @param ticks the desired absolute encoder offset in encoder ticks
   */
  public NovaEncoderConfig setAbsoluteOffset(int ticks) {
    putParameter(NovaParameter.ABS_OFFSET.value, ticks);
    return this;
  }

  public int getAbsoluteOffset() {
    return (int) getParameter(NovaParameter.ABS_OFFSET.value);
  }

  public NovaEncoderConfig setAbsoluteWrapping(boolean isEnabled) {
    putParameter(NovaParameter.ABSOLUTE_WRAPPING.value, isEnabled);
    return this;
  }

  public boolean getAbsoluteWrapping() {
    return (boolean) getParameter(NovaParameter.ABSOLUTE_WRAPPING.value);
  }

  public NovaEncoderConfig setExternalEncoder(ExternalEncoder externalEncoder) {
    putParameter(NovaParameter.EXTERNAL_ENCODER.value, externalEncoder);
    return this;
  }

  public ExternalEncoder getExternalEncoder() {
    return (ExternalEncoder) getParameter(NovaParameter.EXTERNAL_ENCODER.value);
  }

  public NovaEncoderConfig setUsedEncoder(EncoderType encoderType) {
    putParameter(NovaParameter.ENCODER_SELECTED.value, encoderType);
    return this;
  }

  public EncoderType getUsedEncoder() {
    return (EncoderType) getParameter(NovaParameter.ENCODER_SELECTED.value);
  }

  public NovaEncoderConfig setEncoderPosition(double position) {
    putParameter(NovaParameter.ENCODER_POSITION.value, position);
    return this;
  }

  public double getEncoderPosition() {
    return (double) getParameter(NovaParameter.ENCODER_POSITION.value);
  }

  public NovaEncoderConfig apply(NovaEncoderConfig config) {
    this.apply(config);
    return this;
  }
}
