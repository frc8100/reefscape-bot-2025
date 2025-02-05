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

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    /**
     * Zeros the gyro.
     *
     * @param deg The angle to zero the gyro to.
     */
    public default void zeroGyro(double deg) {}

    /**
     * @return The current gyro yaw of the robot.
     */
    @AutoLogOutput(key = "Gyro/Yaw")
    public abstract Rotation2d getYaw();

    // public abstract Rotation2d getRotation2d();

    /**
     * @return The current gyro heading of the robot.
     */
    @AutoLogOutput(key = "Gyro/Heading")
    public abstract Rotation2d getGyroHeading();
}
