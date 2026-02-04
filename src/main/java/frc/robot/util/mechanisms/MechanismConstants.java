package frc.robot.util.mechanisms;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.States.Pos_State;
import frc.robot.util.States.State;

public class MechanismConstants {
  // Physical constraints
  String outputsLogName;
  String tuningLogName;

  // Motor constants
  double moi_kgm2;
  double reduction;
  DCMotor gearbox;
  int[] motorCanIds;
  boolean[] followerInversions;
  MotorType revMotorType;
  SparkMaxConfig sparkConfig;

  // Physical constants
  double mass_kg;
  double codePeriod_s;
  /** For arms */
  double length_m;

  // Profile constants
  State start_State;

  Pos_State min_Pos;
  Pos_State max_Pos;

  /** For arms: Angle of the center of mass above horizontal when the measured angle is zero. */
  Pos_State cmOffset_Pos;

  State limits_State;

  // Feedback constants
  PIDController realPID;
  PIDController simPID;
  SimpleMotorFeedforward realSimpleFF;
  SimpleMotorFeedforward simSimpleFF;
  ArmFeedforward realArmFF;
  ArmFeedforward simArmFF;
  ElevatorFeedforward realElevatorFF;
  ElevatorFeedforward simElevatorFF;
}
