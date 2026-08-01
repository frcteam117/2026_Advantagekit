// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(DrivetrainConstants.PigeonCanId);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final Translation3d zHat = new Translation3d(0, 0, 1);

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(DrivetrainConstants.Chassis.odometryFrequency_Hz);
    StatusSignal.setUpdateFrequencyForAll(
        50,
        pigeon.getAngularVelocityXDevice(),
        pigeon.getAngularVelocityYDevice(),
        pigeon.getAngularVelocityZDevice(),
        pigeon.getAccelerationX(),
        pigeon.getAccelerationY(),
        pigeon.getAccelerationZ());
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = NovaOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        NovaOdometryThread.getInstance().registerSignal(() -> yaw.getValue().in(Radians));
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw).equals(StatusCode.OK);
    inputs.orientation = pigeon.getRotation3d();
    inputs.angularVel[0].mut_replace(pigeon
        .getAngularVelocityXDevice()
        .getValue()); // Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.angularVel[1].mut_replace(pigeon
        .getAngularVelocityYDevice()
        .getValue()); // Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.angularVel[2].mut_replace(pigeon
        .getAngularVelocityZDevice()
        .getValue()); // Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.linearAcc[0].mut_replace(pigeon
        .getAccelerationX()
        .getValue()); // Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.linearAcc[1].mut_replace(pigeon
        .getAccelerationY()
        .getValue()); // Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.linearAcc[2].mut_replace(pigeon
        .getAccelerationZ()
        .getValue()); // Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.angleFromHorizontal.mut_replace(
        Math.acos(zHat.rotateBy(inputs.orientation).dot(zHat)), Radians);

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions = yawPositionQueue.stream()
        .map((Double value) -> Rotation2d.fromRadians(value))
        .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
