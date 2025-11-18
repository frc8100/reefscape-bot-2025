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

package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.util.CoupledYAMSSubsystemIO;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {

        public CoupledYAMSSubsystemIO.SparkMotorControllerDataUnitless driveMotorData =
            CoupledYAMSSubsystemIO.defaultControllerDataUnitless;
        public double driveFFVolts = 0.0;

        public CoupledYAMSSubsystemIO.SparkMotorControllerDataUnitless turnMotorData =
            CoupledYAMSSubsystemIO.defaultControllerDataUnitless;
        public Rotation2d turnAbsolutePosition = new Rotation2d();

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public abstract void updateInputs(ModuleIOInputs inputs);

    /**
     * @return The state of the module.
     */
    public abstract SwerveModuleState getState();

    /** Run the drive motor at the specified open loop value (in volts). */
    public abstract void setDriveOpenLoop(double output);

    /** Run the turn motor at the specified open loop value (in volts). */
    public abstract void setTurnOpenLoop(double output);

    /** Run the drive motor at the specified velocity. Used internally. */
    public abstract void setDriveVelocity(double velocityMetersPerSecond, double driveFeedforwardVoltage);

    /** Run the turn motor to the specified rotation. Used internally. */
    public abstract void setTurnPosition(Rotation2d rotation);

    /**
     * Sets the desired state for the module. Should update the turn and drive motors to reach the desired state.
     * @param desiredState - The desired state for the module.
     * @param currentRotation2d - The current rotation of the module. Used for optimization.
     * @param driveFeedforwardVoltage - The feedforward voltage to apply to the drive motor.
     */
    public abstract void setDesiredState(
        SwerveModuleState desiredState,
        Rotation2d currentRotation2d,
        double driveFeedforwardVoltage
    );

    /**
     * Resets the motor relative angle encoder to the current absolute encoder (CANCoder on real hardware) position reading.
     */
    public default void syncMotorEncoderToAbsoluteEncoder() {}
}
