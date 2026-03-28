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

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.studica.frc.Navx;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final Navx navX = new Navx(0, 100); // rate in Hz
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final Translation3d zHat = new Translation3d(0, 0, 1);

  public GyroIONavX() {
    navX.enableOptionalMessages(
        true, //           comment to make formatter spread this line out
        true, //         comment to make formatter spread this line out
        true, //        comment to make formatter spread this line out
        false, //       comment to make formatter spread this line out
        false, //   comment to make formatter spread this line out
        true, //     comment to make formatter spread this line out
        true, //    comment to make formatter spread this line out
        true, //   comment to make formatter spread this line out
        false, //      comment to make formatter spread this line out
        false); // comment to make formatter spread this line out
    yawTimestampQueue = NovaOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = NovaOdometryThread.getInstance()
        .registerSignal(() -> navX.getRotation2d().getRadians());
  }

  @Override
  public void updateInputs(final GyroIOInputs inputs) {
    // inputs.yaw = Rotation2d.fromDegrees(navX.getYaw().in(Degrees));
    // inputs.yawPosition = Rotation2d.fromDegrees(360);
    inputs.connected = navX.getYaw().in(Radians) < 4.5;
    // inputs.yawVelocityRadPerSec = navX.getAngularVel()[2].in(RadiansPerSecond);
    inputs.orientation = navX.getRotation3d();

    final AngularVelocity[] angVel = navX.getAngularVel();
    // Trust me: their units are wrong in NavX3 version 5.0.5
    inputs.angularVel[0].mut_replace(angVel[0].in(DegreesPerSecond), RadiansPerSecond);
    inputs.angularVel[1].mut_replace(angVel[1].in(DegreesPerSecond), RadiansPerSecond);
    inputs.angularVel[2].mut_replace(angVel[2].in(DegreesPerSecond), RadiansPerSecond);

    final LinearAcceleration[] linAcc = navX.getLinearAccel();
    inputs.linearAcc[0].mut_replace(linAcc[0]);
    inputs.linearAcc[1].mut_replace(linAcc[1]);
    inputs.linearAcc[2].mut_replace(linAcc[2]);

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
