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

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * The IO interface for the gyro. Can be implemented by the real gyro or a simulated gyro.
 */
public interface GyroIO {
    /**
     * The gyro inputs that get updated by the gyro.
     */
    @AutoLog
    public static class GyroIOInputs {

        /**
         * Whether the gyro is connected or not.
         */
        public boolean connected = false;

        /**
         * The yaw position of the gyro. Measured as a Rotation2d.
         * A positive yaw is a counterclockwise rotation.
         */
        public Rotation2d yawPosition = new Rotation2d();

        /**
         * The change in yaw position of the gyro. Measured in radians per second.
         */
        public double yawVelocityRadPerSec = 0.0;

        /**
         * The timestamps of the odometry yaw positions.
         * Example: [0.0, 0.02, 0.04, 0.06, 0.08]
         */
        public double[] odometryYawTimestamps = new double[] {};

        /**
         * The odometry yaw positions. Should correspond with the timestamps {@link #odometryYawTimestamps}.
         */
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    /**
     * Updates the gyro inputs. Automatically called by the robot loop.
     *
     * @param inputs The gyro inputs to update.
     */
    public abstract void updateInputs(GyroIOInputs inputs);

    /**
     * Zeros the gyro.
     *
     * @param deg The angle to zero the gyro to.
     */
    public abstract void zeroGyro(double deg);

    /**
     * Note: this is functionally equivalent to {@link #getGyroHeading()}.
     * @return The current gyro yaw of the robot.
     */
    @AutoLogOutput(key = "Gyro/Yaw")
    public abstract Rotation2d getYaw();

    /**
     * @return The current gyro heading of the robot.
     */
    @AutoLogOutput(key = "Gyro/Heading")
    public abstract Rotation2d getGyroHeading();
}
