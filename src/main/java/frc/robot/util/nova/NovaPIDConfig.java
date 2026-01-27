package frc.robot.util.nova;

public class NovaPIDConfig extends NovaBaseConfig {
  private final boolean is0;

  public NovaPIDConfig(int index) {
    this.is0 = index == 0;
  }

  public NovaPIDConfig setP(double p) {
    putParameter(is0 ? NovaParameter.KP_0.value : NovaParameter.KP_1.value, p);
    return this;
  }

  public double getP() {
    return (double) getParameter(is0 ? NovaParameter.KP_0.value : NovaParameter.KP_1.value);
  }

  public NovaPIDConfig setI(double i) {
    putParameter(is0 ? NovaParameter.KI_0.value : NovaParameter.KI_1.value, i);
    return this;
  }

  public double getI() {
    return (double) getParameter(is0 ? NovaParameter.KI_0.value : NovaParameter.KI_1.value);
  }

  public NovaPIDConfig setD(double d) {
    putParameter(is0 ? NovaParameter.KD_0.value : NovaParameter.KD_1.value, d);
    return this;
  }

  public double getD() {
    return (double) getParameter(is0 ? NovaParameter.KD_0.value : NovaParameter.KD_1.value);
  }

  public NovaPIDConfig setFF(double ff) {
    putParameter(is0 ? NovaParameter.KF_0.value : NovaParameter.KF_1.value, ff);
    return this;
  }

  public double getFF() {
    return (double) getParameter(is0 ? NovaParameter.KF_0.value : NovaParameter.KF_1.value);
  }

  public NovaPIDConfig setPID(double p, double i, double d) {
    setP(p);
    setI(i);
    setD(d);
    return this;
  }

  public double[] getPID() {
    return new double[] {getP(), getI(), getD()};
  }

  public NovaPIDConfig setPIDF(double p, double i, double d, double ff) {
    setPID(p, i, d);
    setFF(ff);
    return this;
  }

  public double[] getPIDF() {
    return new double[] {getP(), getI(), getD(), getFF()};
  }

  // does not exist as of 9/17/2025
  // public NovaPIDConfig setIZone(double iZone) {
  //     putParameter(is0 ? NovaParameter.I_ZONE_0.value : NovaParameter.I_ZONE_1.value, iZone);
  //     return this;
  // }

  public NovaPIDConfig setAccumulatorCap(double accumulatorCap) {
    putParameter(
        is0 ? NovaParameter.ACCUMULATOR_CAP_0.value : NovaParameter.ACCUMULATOR_CAP_1.value,
        accumulatorCap);
    return this;
  }

  public double getAccumulatorCap() {
    return (double) getParameter(
        is0 ? NovaParameter.ACCUMULATOR_CAP_0.value : NovaParameter.ACCUMULATOR_CAP_1.value);
  }

  public NovaPIDConfig setAllowableError(double allowableError) {
    putParameter(
        is0 ? NovaParameter.PID_ERROR_0.value : NovaParameter.PID_ERROR_1.value, allowableError);
    return this;
  }

  public double getAllowableError() {
    return (double)
        getParameter(is0 ? NovaParameter.PID_ERROR_0.value : NovaParameter.PID_ERROR_1.value);
  }

  public NovaPIDConfig apply(NovaPIDConfig config) {
    this.apply(config);
    return this;
  }
}
