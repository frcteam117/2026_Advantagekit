package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemBase1Mech;
import frc.robot.subsystems.intake.IntakeConstants.*;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.mechanisms.MechanismBase;
import frc.robot.util.states.State;
import frc.robot.util.states.bases.PosVel_State;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase1Mech<PosVel_State> {
  private final SparkMax rollerSpark = new SparkMax(Roller.CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder rollerEncoder = rollerSpark.getEncoder();
  private final MutLinearVelocity rollerSurfaceVel = new MutLinearVelocity(0, 0, MetersPerSecond);

  public IntakeSubsystem() {
    super(IntakeConstants.PIVOT_CONFIG);
    rollerSpark.configure(
        Roller.SPARK_MAX_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Pose3d[] getPose3ds() {
    return getIntakePose(getPivotState().pos(Radians));
  }

  public void periodic() {
    super.periodic();
    rollerSurfaceVel.mut_replace(
        rollerEncoder.getVelocity()
            * 2
            * Math.PI
            * Roller.RADIUS.in(Meters)
            / (Roller.REDUCTION * 60),
        MetersPerSecond);
    Logger.recordOutput(Roller.NT_KEY + "/SurfaceVel", rollerSurfaceVel);
  }

  // Pivot

  public void setPivotGoal(State goal_State, int usedProfile, int usedFeedback) {
    setMech0Goal(goal_State, usedProfile, usedFeedback);
  }

  public void setPivotGoal(State goal_State) {
    setMech0Goal(goal_State);
  }

  public Command getPivotSysIdCommand(SysIdType type) {
    return getMech0SysIdCommand(type);
  }

  public PosVel_State getPivotState() {
    return getMech0State();
  }

  public MechanismBase<PosVel_State> getPivot() {
    return getMechanism0();
  }

  // Roller
  public void setRollerSpeed(double speed) {
    rollerSpark.set(speed);
    Logger.recordOutput(Roller.NT_KEY + "/GoalSpeed", speed);
  }

  public LinearVelocity getRollerSurfaceSpeed() {
    return rollerSurfaceVel;
  }

  private Pose3d[] getIntakePose(double rad) {
    // math based on https://www.desmos.com/calculator/8e3rmxmen2
    double g = Math.sqrt(0.081730601 - 0.0338354902642 * Math.cos(rad));
    double ifVar = Math.PI < rad % (2 * Math.PI) ? 1 : -1;
    double theta_a = rad - 0.564035611052;
    double theta_b = 2.55195984092
        - Math.acos((-0.0732526549 - (g * g)) / (-0.558793904763 * g))
        + ifVar * Math.acos((0.0743979216 - (g * g)) / (-0.121100614367 * g));
    double theta_e = 3.8131545094
        + theta_a
        + Math.acos((0.0732526549 - (g * g)) / (-0.138708357355 * g))
        - ifVar * Math.acos((-0.0743979216 - (g * g)) / (-0.558799646743 * g));
    // these are of b_1 - a_1 on the graph
    double x = 0.06006
        + (0.253999673228 * Math.cos(0.462274273714 + theta_b))
        - (0.217012684652 * Math.cos(0.471540446498 + theta_a));
    double y = -0.00769
        + (0.253999673228 * Math.sin(0.462274273714 + theta_b))
        - (0.217012684652 * Math.sin(0.471540446498 + theta_a));
    double h = Math.sqrt(x * x + y * y);
    double atanOfh = Math.atan2(y, x);
    double theta_c =
        atanOfh + 1.22843881587 - Math.acos((-0.0107190074 - h * h) / (-0.510740385715 * h));
    double theta_d =
        atanOfh + 4.7931822977 + Math.acos((0.0107190074 - h * h) / (-0.466882974631 * h));

    Pose3d[] poses = new Pose3d[5];
    // the thetas are negative because pitch rotates down from z to x and the thetas are the angle
    // above the normal position
    poses[0] = new Pose3d(0.16543, 0, 0.18708, new Rotation3d(0, -theta_a, 0));
    poses[1] = new Pose3d(0.22549, 0, 0.17939, new Rotation3d(0, -theta_b, 0));
    poses[2] = new Pose3d(
        new Translation3d(0.19333, 0, 0.09858)
            .rotateBy(poses[0].getRotation())
            .plus(poses[0].getTranslation()),
        new Rotation3d(0, -theta_c, 0));
    poses[3] = new Pose3d(
        new Translation3d(0.22734, 0, 0.11328)
            .rotateBy(poses[1].getRotation())
            .plus(poses[1].getTranslation()),
        new Rotation3d(0, -theta_d, 0));
    poses[4] = new Pose3d(
        new Translation3d(0.25318, 0, 0.11817)
            .rotateBy(poses[0].getRotation())
            .plus(poses[0].getTranslation()),
        new Rotation3d(0, -theta_e, 0));

    return poses;
  }
}
