package frc.robot.util;

public class States {
  public interface StateBase {
    public abstract String getShortName();

    public abstract State getNaNState();
  }

  public interface State extends StateBase {}

  public interface Linear {}

  public interface Angular {}

  public interface Pos_State extends State {
    public abstract double pos();

    public default double position() {
      return pos();
    }
  }

  public interface Vel_State extends State {
    public abstract double vel();

    public default double velocity() {
      return vel();
    }
  }

  public interface Acc_State extends State {
    public abstract double acc();

    public default double acceleration() {
      return acc();
    }
  }

  public interface Jerk_State extends State {
    public abstract double jerk();
  }

  public interface PosVel_State extends Pos_State, Vel_State {}

  public interface PosAcc_State extends Pos_State, Acc_State {}

  public interface PosJerk_State extends Pos_State, Jerk_State {}

  public interface VelAcc_State extends Vel_State, Acc_State {}

  public interface VelJerk_State extends Vel_State, Jerk_State {}

  public interface AccJerk_State extends Acc_State, Jerk_State {}

  public interface PosVelAcc_State extends PosVel_State, PosAcc_State, VelAcc_State {}

  public interface PosVelJerk_State extends PosVel_State, PosJerk_State, VelJerk_State {}

  public interface PosAccJerk_State extends PosAcc_State, PosJerk_State, AccJerk_State {}

  public interface VelAccJerk_State extends VelAcc_State, VelJerk_State, AccJerk_State {}

  public interface PosVelAccJerk_State
      extends PosVelAcc_State, PosVelJerk_State, PosAccJerk_State, VelAccJerk_State {}

  public record Voltage_State(double V) implements State {
    public String getShortName() {
      return "Voltage";
    }

    public State getNaNState() {
      return new Voltage_State(Double.NaN);
    }
  }

  public record Current_State(double A) implements State {
    public String getShortName() {
      return "Current";
    }

    public State getNaNState() {
      return new Current_State(Double.NaN);
    }
  }

  public record LinearP_State(double m) implements Linear, Pos_State {
    public String getShortName() {
      return "LinP";
    }

    public LinearP_State getNaNState() {
      return new LinearP_State(Double.NaN);
    }

    public double pos() {
      return this.m();
    }
  }

  public record LinearV_State(double mPs) implements Linear, Vel_State {
    public String getShortName() {
      return "LinV";
    }

    public LinearV_State getNaNState() {
      return new LinearV_State(Double.NaN);
    }

    public double vel() {
      return this.mPs();
    }
  }

  public record LinearA_State(double mPs2) implements Linear, Acc_State {
    public String getShortName() {
      return "LinA";
    }

    public LinearA_State getNaNState() {
      return new LinearA_State(Double.NaN);
    }

    public double acc() {
      return this.mPs2();
    }
  }

  public record LinearJ_State(double mPs3) implements Linear, Jerk_State {
    public String getShortName() {
      return "LinJ";
    }

    public LinearJ_State getNaNState() {
      return new LinearJ_State(Double.NaN);
    }

    public double jerk() {
      return this.mPs3();
    }
  }

  public record LinearPV_State(double m, double mPs) implements Linear, PosVel_State {
    public String getShortName() {
      return "LinPV";
    }

    public LinearPV_State getNaNState() {
      return new LinearPV_State(Double.NaN, Double.NaN);
    }

    public double pos() {
      return this.m();
    }

    public double vel() {
      return this.mPs();
    }
  }

  public record LinearPA_State(double m, double mPs2) implements Linear, PosAcc_State {
    public String getShortName() {
      return "LinPA";
    }

    public LinearPA_State getNaNState() {
      return new LinearPA_State(Double.NaN, Double.NaN);
    }

    public double pos() {
      return this.m();
    }

    public double acc() {
      return this.mPs2();
    }
  }

  public record LinearPJ_State(double m, double mPs3) implements Linear, PosJerk_State {
    public String getShortName() {
      return "LinPJ";
    }

    public LinearPJ_State getNaNState() {
      return new LinearPJ_State(Double.NaN, Double.NaN);
    }

    public double pos() {
      return this.m();
    }

    public double jerk() {
      return this.mPs3();
    }
  }

  public record LinearVA_State(double mPs, double mPs2) implements Linear, VelAcc_State {
    public String getShortName() {
      return "LinVA";
    }

    public LinearVA_State getNaNState() {
      return new LinearVA_State(Double.NaN, Double.NaN);
    }

    public double vel() {
      return this.mPs();
    }

    public double acc() {
      return this.mPs2();
    }
  }

  public record LinearVJ_State(double mPs, double mPs3) implements Linear, VelJerk_State {
    public String getShortName() {
      return "LinVJ";
    }

    public LinearVJ_State getNaNState() {
      return new LinearVJ_State(Double.NaN, Double.NaN);
    }

    public double vel() {
      return this.mPs();
    }

    public double jerk() {
      return this.mPs3();
    }
  }

  public record LinearAJ_State(double mPs2, double mPs3) implements Linear, AccJerk_State {
    public String getShortName() {
      return "LinAJ";
    }

    public LinearAJ_State getNaNState() {
      return new LinearAJ_State(Double.NaN, Double.NaN);
    }

    public double acc() {
      return this.mPs2();
    }

    public double jerk() {
      return this.mPs3();
    }
  }

  public record LinearPVA_State(double m, double mPs, double mPs2)
      implements Linear, PosVelAcc_State {
    public String getShortName() {
      return "LinPVA";
    }

    public LinearPVA_State getNaNState() {
      return new LinearPVA_State(Double.NaN, Double.NaN, Double.NaN);
    }

    public double pos() {
      return this.m();
    }

    public double vel() {
      return this.mPs();
    }

    public double acc() {
      return this.mPs2();
    }
  }

  public record LinearPVJ_State(double m, double mPs, double mPs3)
      implements Linear, PosVelJerk_State {
    public String getShortName() {
      return "LinPVJ";
    }

    public LinearPVJ_State getNaNState() {
      return new LinearPVJ_State(Double.NaN, Double.NaN, Double.NaN);
    }

    public double pos() {
      return this.m();
    }

    public double vel() {
      return this.mPs();
    }

    public double jerk() {
      return this.mPs3();
    }
  }

  public record LinearPAJ_State(double m, double mPs2, double mPs3)
      implements Linear, PosAccJerk_State {
    public String getShortName() {
      return "LinPAJ";
    }

    public LinearPAJ_State getNaNState() {
      return new LinearPAJ_State(Double.NaN, Double.NaN, Double.NaN);
    }

    public double pos() {
      return this.m();
    }

    public double acc() {
      return this.mPs2();
    }

    public double jerk() {
      return this.mPs3();
    }
  }

  public record LinearVAJ_State(double mPs, double mPs2, double mPs3)
      implements Linear, VelAccJerk_State {
    public String getShortName() {
      return "LinVAJ";
    }

    public LinearVAJ_State getNaNState() {
      return new LinearVAJ_State(Double.NaN, Double.NaN, Double.NaN);
    }

    public double vel() {
      return this.mPs();
    }

    public double acc() {
      return this.mPs2();
    }

    public double jerk() {
      return this.mPs3();
    }
  }

  public record LinearPVAJ_State(double m, double mPs, double mPs2, double mPs3)
      implements Linear, PosVelAccJerk_State {
    public String getShortName() {
      return "LinPVAJ";
    }

    public LinearPVAJ_State getNaNState() {
      return new LinearPVAJ_State(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    }

    public double pos() {
      return this.m();
    }

    public double vel() {
      return this.mPs();
    }

    public double acc() {
      return this.mPs2();
    }

    public double jerk() {
      return this.mPs3();
    }
  }

  public record AngularP_State(double rad) implements Angular, Pos_State {
    public String getShortName() {
      return "AngP";
    }

    public AngularP_State getNaNState() {
      return new AngularP_State(Double.NaN);
    }

    public double pos() {
      return this.rad();
    }
  }

  public record AngularV_State(double radPs) implements Angular, Vel_State {
    public String getShortName() {
      return "AngV";
    }

    public AngularV_State getNaNState() {
      return new AngularV_State(Double.NaN);
    }

    public double vel() {
      return this.radPs();
    }
  }

  public record AngularA_State(double radPs2) implements Angular, Acc_State {
    public String getShortName() {
      return "AngA";
    }

    public AngularA_State getNaNState() {
      return new AngularA_State(Double.NaN);
    }

    public double acc() {
      return this.radPs2();
    }
  }

  public record AngularJ_State(double radPs3) implements Angular, Jerk_State {
    public String getShortName() {
      return "AngJ";
    }

    public AngularJ_State getNaNState() {
      return new AngularJ_State(Double.NaN);
    }

    public double jerk() {
      return this.radPs3();
    }
  }

  public record AngularPV_State(double rad, double radPs) implements Angular, PosVel_State {
    public String getShortName() {
      return "AngPV";
    }

    public AngularPV_State getNaNState() {
      return new AngularPV_State(Double.NaN, Double.NaN);
    }

    public double pos() {
      return this.rad();
    }

    public double vel() {
      return this.radPs();
    }
  }

  public record AngularPA_State(double rad, double radPs2) implements Angular, PosAcc_State {
    public String getShortName() {
      return "AngPA";
    }

    public AngularPA_State getNaNState() {
      return new AngularPA_State(Double.NaN, Double.NaN);
    }

    public double pos() {
      return this.rad();
    }

    public double acc() {
      return this.radPs2();
    }
  }

  public record AngularPJ_State(double rad, double radPs3) implements Angular, PosJerk_State {
    public String getShortName() {
      return "AngPJ";
    }

    public AngularPJ_State getNaNState() {
      return new AngularPJ_State(Double.NaN, Double.NaN);
    }

    public double pos() {
      return this.rad();
    }

    public double jerk() {
      return this.radPs3();
    }
  }

  public record AngularVA_State(double radPs, double radPs2) implements Angular, VelAcc_State {
    public String getShortName() {
      return "AngVA";
    }

    public AngularVA_State getNaNState() {
      return new AngularVA_State(Double.NaN, Double.NaN);
    }

    public double vel() {
      return this.radPs();
    }

    public double acc() {
      return this.radPs2();
    }
  }

  public record AngularVJ_State(double radPs, double radPs3) implements Angular, VelJerk_State {
    public String getShortName() {
      return "AngVJ";
    }

    public AngularVJ_State getNaNState() {
      return new AngularVJ_State(Double.NaN, Double.NaN);
    }

    public double vel() {
      return this.radPs();
    }

    public double jerk() {
      return this.radPs3();
    }
  }

  public record AngularAJ_State(double radPs2, double radPs3) implements Angular, AccJerk_State {
    public String getShortName() {
      return "AngAJ";
    }

    public AngularAJ_State getNaNState() {
      return new AngularAJ_State(Double.NaN, Double.NaN);
    }

    public double acc() {
      return this.radPs2();
    }

    public double jerk() {
      return this.radPs3();
    }
  }

  public record AngularPVA_State(double rad, double radPs, double radPs2)
      implements Angular, PosVelAcc_State {
    public String getShortName() {
      return "AngPVA";
    }

    public AngularPVA_State getNaNState() {
      return new AngularPVA_State(Double.NaN, Double.NaN, Double.NaN);
    }

    public double pos() {
      return this.rad();
    }

    public double vel() {
      return this.radPs();
    }

    public double acc() {
      return this.radPs2();
    }
  }

  public record AngularPVJ_State(double rad, double radPs, double radPs3)
      implements Angular, PosVelJerk_State {
    public String getShortName() {
      return "AngPVJ";
    }

    public AngularPVJ_State getNaNState() {
      return new AngularPVJ_State(Double.NaN, Double.NaN, Double.NaN);
    }

    public double pos() {
      return this.rad();
    }

    public double vel() {
      return this.radPs();
    }

    public double jerk() {
      return this.radPs3();
    }
  }

  public record AngularPAJ_State(double rad, double radPs2, double radPs3)
      implements Angular, PosAccJerk_State {
    public String getShortName() {
      return "AngPAJ";
    }

    public AngularPAJ_State getNaNState() {
      return new AngularPAJ_State(Double.NaN, Double.NaN, Double.NaN);
    }

    public double pos() {
      return this.rad();
    }

    public double acc() {
      return this.radPs2();
    }

    public double jerk() {
      return this.radPs3();
    }
  }

  public record AngularVAJ_State(double radPs, double radPs2, double radPs3)
      implements Angular, VelAccJerk_State {
    public String getShortName() {
      return "AngVAJ";
    }

    public AngularVAJ_State getNaNState() {
      return new AngularVAJ_State(Double.NaN, Double.NaN, Double.NaN);
    }

    public double vel() {
      return this.radPs();
    }

    public double acc() {
      return this.radPs2();
    }

    public double jerk() {
      return this.radPs3();
    }
  }

  public record AngularPVAJ_State(double rad, double radPs, double radPs2, double radPs3)
      implements Angular, PosVelAccJerk_State {
    public String getShortName() {
      return "AngPVAJ";
    }

    public AngularPVAJ_State getNaNState() {
      return new AngularPVAJ_State(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    }

    public double pos() {
      return this.rad();
    }

    public double vel() {
      return this.radPs();
    }

    public double acc() {
      return this.radPs2();
    }

    public double jerk() {
      return this.radPs3();
    }
  }
}
