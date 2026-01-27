package frc.robot.util.nova;

public class NovaCANConfig extends NovaBaseConfig {

  public NovaCANConfig setFaultPeriod(double period_s) {
    putParameter(NovaParameter.FREQ_FAULT.value, period_s);
    return this;
  }

  public double getFaultPeriod() {
    return (double) getParameter(NovaParameter.FREQ_FAULT.value);
  }

  public NovaCANConfig setSensorPeriod(double period_s) {
    putParameter(NovaParameter.FREQ_SENSOR.value, period_s);
    return this;
  }

  public double getSensorPeriod() {
    return (double) getParameter(NovaParameter.FREQ_SENSOR.value);
  }

  public NovaCANConfig setQuadSensorPeriod(double period_s) {
    putParameter(NovaParameter.FREQ_QUAD_SENSOR.value, period_s);
    return this;
  }

  public double getQuadSensorPeriod() {
    return (double) getParameter(NovaParameter.FREQ_QUAD_SENSOR.value);
  }

  public NovaCANConfig setControlPeriod(double period_s) {
    putParameter(NovaParameter.FREQ_CONTROL.value, period_s);
    return this;
  }

  public double getControlPeriod() {
    return (double) getParameter(NovaParameter.FREQ_CONTROL.value);
  }

  public NovaCANConfig setCurrentPeriod(double period_s) {
    putParameter(NovaParameter.FREQ_CURRENT.value, period_s);
    return this;
  }

  public double getCurrentPeriod() {
    return (double) getParameter(NovaParameter.FREQ_CURRENT.value);
  }

  public NovaCANConfig apply(NovaCANConfig config) {
    this.apply(config);
    return this;
  }
}
