package frc.robot.util.nova;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.PIDSlot;

public class NovaConfig extends NovaBaseConfig {
  public final NovaEncoderConfig encoder;
  public final NovaLimitsConfig limits;
  public final NovaCANConfig canFreq;
  public final NovaPIDConfig pid0;
  public final NovaPIDConfig pid1;

  public NovaConfig() {
    encoder = new NovaEncoderConfig();
    limits = new NovaLimitsConfig();
    canFreq = new NovaCANConfig();
    pid0 = new NovaPIDConfig(0);
    pid1 = new NovaPIDConfig(1);

    this.setFactoryReset(true);
  }

  public enum BrakeMode {
    COAST,
    BRAKE
  }

  public NovaConfig setInversion(boolean isInverted) {
    putParameter(NovaParameter.INVERTED.value, isInverted);
    return this;
  }

  public boolean getInversion() {
    return (boolean) getParameter(NovaParameter.INVERTED.value);
  }

  public NovaConfig setVoltageCompensation(double vcomp) {
    putParameter(NovaParameter.VOLTAGE_COMP.value, vcomp);
    return this;
  }

  public double getVoltageCompensation() {
    return (double) getParameter(NovaParameter.VOLTAGE_COMP.value);
  }

  public NovaConfig setBrakeMode(BrakeMode brakeMode) {
    putParameter(NovaParameter.BRAKE_MODE.value, brakeMode == BrakeMode.COAST ? false : true);
    return this;
  }

  public BrakeMode getBrakeMode() {
    return (BrakeMode) getParameter(NovaParameter.BRAKE_MODE.value);
  }

  public NovaConfig setTemperatureThrottleEnable(boolean isEnabled) {
    putParameter(NovaParameter.TEMP_THROTTLE.value, isEnabled);
    return this;
  }

  public boolean getTemperatureThrottleEnable() {
    return (boolean) getParameter(NovaParameter.TEMP_THROTTLE.value);
  }

  public NovaConfig setUsedPIDSlot(PIDSlot pidSlot) {
    putParameter(NovaParameter.PID_SLOT.value, pidSlot);
    return this;
  }

  public PIDSlot getUsedPIDSlot() {
    return (PIDSlot) getParameter(NovaParameter.PID_SLOT.value);
  }

  public NovaConfig setNTLogging(boolean isEnabled) {
    putParameter(NovaParameter.NT_LOGGING.value, isEnabled);
    return this;
  }

  public boolean getNTLogging() {
    return (boolean) getParameter(NovaParameter.NT_LOGGING.value);
  }

  /** Defaults to resetting the ThriftyNova when configured (true) */
  public NovaConfig setFactoryReset(boolean willReset) {
    putParameter(NovaParameter.FACTORY_RESET.value, willReset);
    return this;
  }

  public boolean getFactoryReset() {
    return (boolean) getParameter(NovaParameter.FACTORY_RESET.value);
  }

  /** Updates depend on the fault status frame */
  public NovaConfig follow(int canID) {
    putParameter(NovaParameter.FOLLOWER_ID.value, canID);
    return this;
  }

  public NovaConfig apply(NovaConfig config) {
    super.apply(config);
    encoder.apply(config.encoder);
    limits.apply(config.limits);
    canFreq.apply(config.canFreq);
    pid0.apply(config.pid0);
    pid1.apply(config.pid1);
    return this;
  }

  public NovaConfig apply(NovaEncoderConfig config) {
    encoder.apply(config);
    return this;
  }

  public NovaConfig apply(NovaLimitsConfig config) {
    limits.apply(config);
    return this;
  }

  public NovaConfig apply(NovaCANConfig config) {
    canFreq.apply(config);
    return this;
  }

  @Override
  public void configure(ThriftyNova nova) {
    if ((boolean) getParameter(NovaParameter.FACTORY_RESET.value)) {
      nova.factoryReset();
    }
    try {
      Thread.sleep(30);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    super.configure(nova);
    encoder.configure(nova);
    limits.configure(nova);
    canFreq.configure(nova);
    pid0.configure(nova);
    pid1.configure(nova);
  }
}
