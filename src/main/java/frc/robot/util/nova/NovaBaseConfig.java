package frc.robot.util.nova;

import static com.thethriftybot.devices.ThriftyNova.Error.*;
import static frc.robot.util.nova.NovaUtil.tryUntilOk;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.CurrentType;
import com.thethriftybot.devices.ThriftyNova.EncoderType;
import com.thethriftybot.devices.ThriftyNova.ExternalEncoder;
import com.thethriftybot.devices.ThriftyNova.MotorType;
import com.thethriftybot.devices.ThriftyNova.PIDSlot;
import java.util.HashMap;
import java.util.Map;

public abstract class NovaBaseConfig {
  protected int maxAttempts = 3;
  private Map<Integer, Object> parameters = new HashMap<>();

  protected void putParameter(int parameterId, Object value) {
    parameters.put(parameterId, value);
  }

  protected Object getParameter(int parameterId) {
    return parameters.get(parameterId);
  }

  protected Object getParameter(NovaBaseConfig fromConfig, int parameterId) {
    return fromConfig.getParameter(parameterId);
  }

  protected void removeParameter(int parameterId) {
    parameters.remove(parameterId);
  }

  protected void removeParameter(NovaBaseConfig fromConfig, int parameterId) {
    fromConfig.removeParameter(parameterId);
  }

  protected void apply(NovaBaseConfig config) {
    for (Map.Entry<Integer, Object> parameter : config.parameters.entrySet()) {
      putParameter(parameter.getKey(), parameter.getValue());
    }
  }

  protected void configure(ThriftyNova nova) {
    for (Map.Entry<Integer, Object> parameter : this.parameters.entrySet()) {
      switch (parameter.getKey()) {
        case 0 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.setInversion((boolean) parameter.getValue()),
              () -> {
                return nova.getInverted() == (boolean) parameter.getValue();
              },
              "Successfully configured SET_INVERSION on ThriftyNova with CAN Id: ",
              "Failed to configure SET_INVERSION on ThriftyNova with CAN Id: ",
              SET_INVERSION);
        case 1 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.setBrakeMode((boolean) parameter.getValue()),
              () -> {
                return nova.getBrakeMode() == (boolean) parameter.getValue();
              },
              "Successfully configured SET_BRAKE_MODE on ThriftyNova with CAN Id: ",
              "Failed to configure SET_BRAKE_MODE on ThriftyNova with CAN Id: ",
              SET_BRAKE_MODE);
        case 2 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () ->
                  nova.setMaxOutput((double) parameter.getValue(), (double) this.parameters.get(3)),
              () -> {
                return nova.getMaxForwardOutput() == (double) parameter.getValue()
                    && nova.getMaxReverseOutput() == (double) this.parameters.get(3);
              },
              "Successfully configured SET_MAX_FWD, SET_MAX_REV on ThriftyNova with CAN Id: ",
              "Failed to configure SET_MAX_FWD, SET_MAX_REV on ThriftyNova with CAN Id: ",
              SET_MAX_FWD,
              SET_MAX_REV);
        case 4 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.setRampUp((double) parameter.getValue()),
              () -> {
                return nova.getRampUp() == (double) parameter.getValue();
              },
              "Successfully configured SET_RAMP_UP on ThriftyNova with CAN Id: ",
              "Failed to configure SET_RAMP_UP on ThriftyNova with CAN Id: ",
              SET_RAMP_UP);
        case 5 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.setRampDown((double) parameter.getValue()),
              () -> {
                return nova.getRampDown() == (double) parameter.getValue();
              },
              "Successfully configured SET_RAMP_DOWN on ThriftyNova with CAN Id: ",
              "Failed to configure SET_RAMP_DOWN on ThriftyNova with CAN Id: ",
              SET_RAMP_DOWN);
        case 6 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.setMaxCurrent(CurrentType.STATOR, (double) parameter.getValue()),
              () -> {
                return nova.getMaxCurrent() == (double) parameter.getValue();
              },
              "Successfully configured SET_MAX_CURRENT (Stator) on ThriftyNova with CAN Id: ",
              "Failed to configure SET_MAX_CURRENT (Stator) on ThriftyNova with CAN Id: ",
              SET_MAX_CURRENT);
        case 7 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.setMaxCurrent(CurrentType.SUPPLY, (double) parameter.getValue()),
              () -> {
                return nova.getMaxCurrent() == (double) parameter.getValue();
              },
              "Successfully configured SET_MAX_CURRENT (Supply) on ThriftyNova with CAN Id: ",
              "Failed to configure SET_MAX_CURRENT (Supply) on ThriftyNova with CAN Id: ",
              SET_MAX_CURRENT);
        case 8 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.follow((int) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_FOLLOWER_ID on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_FOLLOWER_ID on ThriftyNova with CAN Id: ",
              SET_FOLLOWER_ID);
        case 9 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.pid0.setP((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_KP_0 on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_KP_0 on ThriftyNova with CAN Id: ",
              SET_KP_0);
        case 10 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.pid0.setI((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_KI_0 on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_KI_0 on ThriftyNova with CAN Id: ",
              SET_KI_0);
        case 11 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.pid0.setD((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_KD_0 on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_KD_0 on ThriftyNova with CAN Id: ",
              SET_KD_0);
        case 12 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.pid0.setFF((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_KF_0 on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_KF_0 on ThriftyNova with CAN Id: ",
              SET_KF_0);
        case 13 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.pid0.setAccumulatorCap((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_IZONE_0 on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_IZONE_0 on ThriftyNova with CAN Id: ",
              SET_IZONE_0);
        case 14 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.pid0.setAllowableError((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_PID_ERROR_0 on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_PID_ERROR_0 on ThriftyNova with CAN Id: ",
              SET_PID_ERROR_0);
        case 15 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.pid1.setP((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_KP_1 on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_KP_1 on ThriftyNova with CAN Id: ",
              SET_KP_1);
        case 16 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.pid1.setI((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_KI_1 on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_KI_1 on ThriftyNova with CAN Id: ",
              SET_KI_1);
        case 17 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.pid1.setD((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_KD_1 on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_KD_1 on ThriftyNova with CAN Id: ",
              SET_KD_1);
        case 18 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.pid1.setFF((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_KF_1 on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_KF_1 on ThriftyNova with CAN Id: ",
              SET_KF_1);
        case 19 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.pid1.setAccumulatorCap((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_IZONE_1 on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_IZONE_1 on ThriftyNova with CAN Id: ",
              SET_IZONE_1);
        case 20 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.pid1.setAllowableError((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_PID_ERROR_1 on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_PID_ERROR_1 on ThriftyNova with CAN Id: ",
              SET_PID_ERROR_1);
        case 21 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.canFreq.setFault((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_FREQ_FAULT on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_FREQ_FAULT on ThriftyNova with CAN Id: ",
              SET_FREQ_FAULT);
        case 22 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.canFreq.setSensor((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_FREQ_SENSOR on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_FREQ_SENSOR on ThriftyNova with CAN Id: ",
              SET_FREQ_SENSOR);
        case 23 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.canFreq.setQuadSensor((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_FREQ_QUAD_SENSOR on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_FREQ_QUAD_SENSOR on ThriftyNova with CAN Id: ",
              SET_FREQ_QUAD_SENSOR);
        case 24 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.canFreq.setControl((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_FREQ_CTRL on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_FREQ_CTRL on ThriftyNova with CAN Id: ",
              SET_FREQ_CTRL);
        case 25 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.canFreq.setCurrent((double) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of SET_FREQ_CURRENT on ThriftyNova with CAN Id: ",
              "Unknown configuration status of SET_FREQ_CURRENT on ThriftyNova with CAN Id: ",
              SET_FREQ_CURRENT);
        case 26 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.setSoftLimits(
                  (double) parameters.getOrDefault(27, -(double) parameter.getValue()),
                  (double) parameter.getValue()),
              () -> {
                return nova.getForwardSoftLimit() == (double) parameter.getValue()
                    && nova.getReverseSoftLimit()
                        == (double) parameters.getOrDefault(27, -(double) parameter.getValue());
              },
              "Successfully configured SET_SOFT_LIMIT_FWD, SET_SOFT_LIMIT_REV on ThriftyNova with CAN Id: ",
              "Failed to configure SET_SOFT_LIMIT_FWD, SET_SOFT_LIMIT_REV on ThriftyNova with CAN Id: ",
              SET_SOFT_LIMIT_FWD,
              SET_SOFT_LIMIT_REV);
        case 28 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.enableSoftLimits((boolean) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of ENABLE_SOFT_LIMIT on ThriftyNova with CAN Id: ",
              "Unknown configuration status of ENABLE_SOFT_LIMIT on ThriftyNova with CAN Id: ",
              ENABLE_SOFT_LIMIT);
        case 29 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.enableHardLimits((boolean) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of ENABLE_HARD_LIMIT on ThriftyNova with CAN Id: ",
              "Unknown configuration status of ENABLE_HARD_LIMIT on ThriftyNova with CAN Id: ",
              ENABLE_HARD_LIMIT);
        case 30 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.setAbsOffset((int) parameter.getValue()),
              () -> {
                return nova.getAbsOffset() == (double) parameter.getValue();
              },
              "Successfully configured SET_ABS_OFFSET on ThriftyNova with CAN Id: ",
              "Failed to configure SET_ABS_OFFSET on ThriftyNova with CAN Id: ",
              SET_ABS_OFFSET);
        case 31 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.setVoltageCompensation((double) parameter.getValue()),
              () -> {
                return nova.getVoltageCompensationEnabled() == (double) parameter.getValue();
              },
              "Successfully configured SET_VOLTAGE_COMP on ThriftyNova with CAN Id: ",
              "Failed to configure SET_VOLTAGE_COMP on ThriftyNova with CAN Id: ",
              SET_VOLTAGE_COMP);
        case 32 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.setExternalEncoder((ExternalEncoder) parameter.getValue()),
              () -> {
                return true;
              },
              "Unknown configuration status of EXTERNAL_ENCODER_SELECT on ThriftyNova with CAN Id: ",
              "Unknown configuration status of EXTERNAL_ENCODER_SELECT on ThriftyNova with CAN Id: ",
              EXTERNAL_ENCODER_SELECT);
        case 33 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.setTempThrottleEnable((boolean) parameter.getValue()),
              () -> {
                return nova.getTempThrottleEnable() == (boolean) parameter.getValue();
              },
              "Successfully configured ENABLE_TEMP_THROTTLE on ThriftyNova with CAN Id: ",
              "Failed to configure ENABLE_TEMP_THROTTLE on ThriftyNova with CAN Id: ",
              ENABLE_TEMP_THROTTLE);
        case 34 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.setMotorType((MotorType) parameter.getValue()),
              () -> {
                return nova.getMotorType() == (MotorType) parameter.getValue();
              },
              "Successfully configured SET_MOTOR_TYPE on ThriftyNova with CAN Id: ",
              "Failed to configure SET_MOTOR_TYPE on ThriftyNova with CAN Id: ",
              SET_MOTOR_TYPE);
        case 36 ->
          tryUntilOk(
              nova,
              maxAttempts,
              () -> nova.setAbsoluteWrapping((boolean) parameter.getValue()),
              () -> {
                return nova.getAbsoluteWrapping() == (boolean) parameter.getValue();
              },
              "Successfully configured SET_ABSOLUTE_WRAPPING on ThriftyNova with CAN Id: ",
              "Failed to configure SET_ABSOLUTE_WRAPPING on ThriftyNova with CAN Id: ",
              SET_ABSOLUTE_WRAPPING);
        case 37 -> nova.usePIDSlot((PIDSlot) parameter.getValue());
        case 38 -> nova.setNTLogging((boolean) parameter.getValue());
        case 39 -> nova.useEncoderType((EncoderType) parameter.getValue());
        case 40 -> nova.setEncoderPosition((double) parameter.getValue());
      }
    }
  }
}
