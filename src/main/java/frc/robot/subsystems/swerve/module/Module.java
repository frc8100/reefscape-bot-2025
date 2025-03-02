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

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.robot.subsystems.swerve.SwerveConfig;
import org.littletonrobotics.junction.Logger;

/**
 * A single swerve module, including a drive motor and a turn motor.
 * Includes an abstracted IO interface for easy swapping of hardware/sim.
 */
public class Module {

    /**
     * The IO interface for a swerve module.
     * Get inputs from the module and set outputs to the module.
     */
    private final ModuleIO io;

    /** The inputs to the module. */
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    /** The index of the module. */
    public final int index;

    /** Alerts for disconnected motors. */
    private final Alert driveDisconnectedAlert;

    private final Alert turnDisconnectedAlert;

    /** The odometry positions received this cycle. This is processed in {@link SparkSwerveOdometryThread}. */
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    /** Creates a new module with the given IO and index. */
    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
        driveDisconnectedAlert = new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError
        );
        turnDisconnectedAlert = new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".",
            AlertType.kError
        );
    }

    /** Updates the inputs to the module. */
    public void periodic() {
        // Update inputs
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        // Calculate positions for odometry

        // All signals are sampled together
        int sampleCount = inputs.odometryTimestamps.length;
        odometryPositions = new SwerveModulePosition[sampleCount];

        // For each sample, convert the drive position to meters and pair it with the turn angle
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * SwerveConfig.WHEEL_RADIUS.in(Meters);
            Rotation2d angle = inputs.odometryTurnPositions[i];

            // Store the position
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint

        // CTREModuleState functions for any motor type
        state = CTREModuleState.optimize(state, inputs.turnPosition);

        // Apply setpoints
        // io.setDriveVelocity(state.speedMetersPerSecond / wheelRadiusMeters);
        // io.setTurnPosition(state.angle);
        // io.setDriveVelocity(state);
        // io.setTurnPosition(state);
        io.setDesiredState(state);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(new Rotation2d());
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePositionRad * SwerveConfig.WHEEL_RADIUS.in(Meters);
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * SwerveConfig.WHEEL_RADIUS.in(Meters);
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    /** Returns the module velocity in rad/sec. */
    public double getFFCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }
}
