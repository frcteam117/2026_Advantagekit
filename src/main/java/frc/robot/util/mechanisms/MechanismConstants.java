package frc.robot.util.mechanisms;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.StateUtil.Pos_State;
import frc.robot.util.StateUtil.State;

public class MechanismConstants {
  // Physical constraints
  public String outputsLogName;
  public String tuningLogName;

  // Motor constants
  public String[] motorNames;
  public Double moi_kgm2;
  public Double reduction;
  public DCMotor gearbox;
  public int[] motorCanIds;
  /** true if that motor should follow opposite of the first motor */
  public boolean[] followerInversions;

  public MotorType baseRevMotorType;
  public MotorType[] revMotorTypes;
  public MotorType revMotorType;
  public SparkMaxConfig baseSparkConfig;
  public SparkMaxConfig[] sparkConfigs;
  public ResetMode revResetMode = ResetMode.kResetSafeParameters;
  public PersistMode revPersistMode = PersistMode.kPersistParameters;

  // Physical constants
  public Double mass_kg;
  public Double codePeriod_s;
  /** For arms */
  public Double length_m;

  // Profile constants
  public State start_State;

  public Pos_State min_Pos;
  public Pos_State max_Pos;
  public Boolean isContinuous;

  /** For arms: Angle of the center of mass above horizontal when the measured angle is zero. */
  public Pos_State cmOffset_Pos;

  public State limits_State;

  // Feedback constants
  public PIDController pid;
  public SimpleMotorFeedforward simpleFF;
  public ArmFeedforward armFF;
  public ElevatorFeedforward elevatorFF;
  // public PIDController realPID;
  // public PIDController simPID;
  // public SimpleMotorFeedforward realSimpleFF;
  // public SimpleMotorFeedforward simSimpleFF;
  // public ArmFeedforward realArmFF;
  // public ArmFeedforward simArmFF;
  // public ElevatorFeedforward realElevatorFF;
  // public ElevatorFeedforward simElevatorFF;
}
