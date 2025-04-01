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

package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.swerve.SparkOdometryThread;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {

    private final Pigeon2 pigeon = new Pigeon2(SwerveConstants.PIGEON_ID);
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

    private Rotation2d yawOffset = new Rotation2d();

    public GyroIOPigeon2() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(SwerveConfig.ODOMETRY_FREQUENCY_HZ);
        yawVelocity.setUpdateFrequency(SwerveConfig.STATUS_SIGNAL_FREQUENCY_HZ);
        pigeon.optimizeBusUtilization();
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue
            .stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public void zeroGyro(double deg) {
        // Invert the gyro if necessary
        if (SwerveConfig.IS_GYRO_INVERTED) {
            deg = -deg;
        }

        // Zero the gyro and update the odometry
        pigeon.setYaw(deg);
    }

    @Override
    public void zeroFieldRelativeGyro(double deg) {
        // Invert the gyro if necessary
        if (SwerveConfig.IS_GYRO_INVERTED) {
            deg = -deg;
        }

        // Set the angle offset
        // pigeon.setYaw(deg);

        yawOffset = getGyroHeading().minus(Rotation2d.fromDegrees(deg));
    }

    @Override
    public Rotation2d getGyroHeading() {
        // If the gyro is inverted, return the inverted yaw
        if (SwerveConfig.IS_GYRO_INVERTED) {
            return Rotation2d.fromDegrees(yaw.getValueAsDouble()).rotateBy(new Rotation2d(180));
        }

        // Otherwise, return the yaw as-is
        return Rotation2d.fromDegrees(yaw.getValueAsDouble());
    }

    @Override
    public Rotation2d getGyroHeadingForFieldRelative() {
        return getGyroHeading().minus(yawOffset);
    }
}
