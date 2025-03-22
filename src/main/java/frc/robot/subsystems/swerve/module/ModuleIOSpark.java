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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.lib.util.SparkUtil;
import frc.lib.util.TunableValue;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import frc.robot.subsystems.swerve.SparkOdometryThread;
import frc.robot.subsystems.swerve.SwerveConfig;
import java.util.Queue;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {

    public final int moduleNumber;

    /** The angle offset. Used to zero the module to a specific angle. */
    private final Rotation2d angleOffset;

    /**
     * The angle motor. This motor is used to control the angle of the module.
     * Includes the integrated encoder {@link #relAngleEncoder} and an absolute CANcoder {@link #angleCANcoder}.
     */
    private final SparkMax angleMotor;

    private final CANcoder angleCANcoder;
    private final RelativeEncoder relAngleEncoder;
    private final SparkClosedLoopController angleClosedLoopController;

    /**
     * The drive motor. This motor is used to control the speed of the module.
     * Includes an integrated encoder {@link #relDriveEncoder}.
     */
    private final SparkMax driveMotor;

    private final RelativeEncoder relDriveEncoder;
    private final SparkClosedLoopController driveClosedLoopController;

    /**
     * A PID controller that uses CANCoder angles exclusively, in degrees.
     * Only used in mod 1.
     */
    private final PIDController debugAnglePidController;

    /**
     * Status signal queue from CANcoder
     */
    private final StatusSignal<Angle> turnAbsolutePosition;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    /**
     * Creates a new Swerve Module.
     *
     * @param moduleNumber The module number.
     * @param moduleConstants The module constants.
     */
    public ModuleIOSpark(int moduleNumber, RevSwerveModuleConstants moduleConstants) {
        // Set the module number and angle offset
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        // Create and configure the angle motor
        angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        relAngleEncoder = angleMotor.getEncoder();
        angleClosedLoopController = angleMotor.getClosedLoopController();
        SparkUtil.tryUntilOk(angleMotor, 5, () ->
            angleMotor.configure(
                SwerveConfig.getAngleMotorConfig(),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            )
        );

        // Create and configure the drive motor
        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        relDriveEncoder = driveMotor.getEncoder();
        driveClosedLoopController = driveMotor.getClosedLoopController();
        SparkUtil.tryUntilOk(driveMotor, 5, () ->
            driveMotor.configure(
                SwerveConfig.getDriveMotorConfig(),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            )
        );
        SparkUtil.tryUntilOk(driveMotor, 5, () -> relDriveEncoder.setPosition(0.0));

        // Create and configure the CANCoder
        angleCANcoder = new CANcoder(moduleConstants.cancoderID);
        angleCANcoder.getConfigurator().refresh(new CANcoderConfiguration());
        angleCANcoder.getConfigurator().apply(SwerveConfig.getCANcoderConfig());

        turnAbsolutePosition = angleCANcoder.getAbsolutePosition();
        turnAbsolutePosition.setUpdateFrequency(50.0);
        angleCANcoder.optimizeBusUtilization();

        // Reset the module to absolute position
        resetToAbsolute();

        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveMotor, relDriveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance()
            .registerSignal(angleMotor, () -> turnAbsolutePosition.getValue().in(Radians));

        TunableValue.addRefreshConfigConsumer(this::onRefresh);

        debugAnglePidController = new PIDController(0.2, SwerveConfig.angleKI, SwerveConfig.angleKD);
        debugAnglePidController.enableContinuousInput(-180, 180);
        debugAnglePidController.setSetpoint(getCanCoder().getDegrees());
    }

    private void onRefresh() {
        var newConfig = SwerveConfig.getDriveMotorConfig();

        newConfig.closedLoop.p(SwerveConfig.driveKPTunable.get()).d(SwerveConfig.driveKDTunable.get());

        driveMotor.configure(newConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * @return The angle of the module.
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
    }

    /**
     * @return The CANCoder angle of the module.
     * ! IMPORTANT: Does not include angle offset
     */
    public Rotation2d getCanCoder() {
        return new Rotation2d(turnAbsolutePosition.getValue().in(Radians));
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(relDriveEncoder.getVelocity(), getAngle());
    }

    /**
     * @return The position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(relDriveEncoder.getPosition(), getAngle());
    }

    /**
     * Resets the module to absolute position. This is used to zero the module to a specific angle.
     * It is also called when the robot is enabled to reset the module to the absolute position.
     */
    private void resetToAbsolute() {
        double absolutePosition = getCanCoder().minus(angleOffset).getDegrees();
        relAngleEncoder.setPosition(absolutePosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        SparkUtil.sparkStickyFault = false;
        SparkUtil.ifOk(driveMotor, relDriveEncoder::getPosition, value -> inputs.drivePositionRad = value);
        SparkUtil.ifOk(driveMotor, relDriveEncoder::getVelocity, value -> inputs.driveVelocityRadPerSec = value);
        SparkUtil.ifOk(
            driveMotor,
            new DoubleSupplier[] { driveMotor::getAppliedOutput, driveMotor::getBusVoltage },
            values -> inputs.driveAppliedVolts = values[0] * values[1]
        );
        SparkUtil.ifOk(driveMotor, driveMotor::getOutputCurrent, value -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

        // Update turn inputs

        // Refresh
        turnAbsolutePosition.refresh();

        SparkUtil.sparkStickyFault = false;
        SparkUtil.ifOk(
            angleMotor,
            () -> turnAbsolutePosition.getValue().in(Radians),
            value -> inputs.turnPosition = new Rotation2d(value).minus(angleOffset)
        );
        SparkUtil.ifOk(
            angleMotor,
            () -> turnAbsolutePosition.getValue().in(Radians),
            value -> inputs.turnPositionRaw = value
        );
        SparkUtil.ifOk(angleMotor, relAngleEncoder::getVelocity, value -> inputs.turnVelocityRadPerSec = value);
        SparkUtil.ifOk(
            angleMotor,
            new DoubleSupplier[] { angleMotor::getAppliedOutput, angleMotor::getBusVoltage },
            values -> inputs.turnAppliedVolts = values[0] * values[1]
        );
        SparkUtil.ifOk(angleMotor, angleMotor::getOutputCurrent, value -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue
            .stream()
            .map((Double value) -> new Rotation2d(value).minus(angleOffset))
            .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveMotor.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        angleMotor.setVoltage(output);
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        setAngle(desiredState);
        setSpeed(desiredState);
    }

    @Override
    public void setDriveVelocity(double speedMetersPerSecond) {
        setSpeed(new SwerveModuleState(speedMetersPerSecond, new Rotation2d()));
    }

    /**
     * Sets the speed of the module.
     * @param desiredState The desired state.
     * @param isOpenLoop Whether the module is in open loop.
     */
    private void setSpeed(SwerveModuleState desiredState) {
        // Calculate the percent output and set the speed
        // double percentOutput = desiredState.speedMetersPerSecond / SwerveConfig.MAX_SPEED.in(MetersPerSecond);

        // Clamp the percent output to the max speed
        // percentOutput = MathUtil.clamp(percentOutput, -SwerveConfig.MAX_DRIVE_POWER, SwerveConfig.MAX_DRIVE_POWER);

        // driveMotor.set(percentOutput);
        // TODO: set the speed using the PID controller
        double velocityRadiansPerSecond = desiredState.speedMetersPerSecond / SwerveConfig.WHEEL_RADIUS.in(Meters);

        double ffVolts =
            SwerveConfig.driveKS * Math.signum(velocityRadiansPerSecond) +
            SwerveConfig.driveKV * velocityRadiansPerSecond;

        // Logger.recordOutput("Swerve/Mod" + Integer.toString(moduleNumber) + "Percent", percentOutput);
        // Logger.recordOutput("Swerve/Mod" + Integer.toString(moduleNumber) + "DrivePID", velocityRadiansPerSecond);
        // Logger.recordOutput("Swerve/Mod" + Integer.toString(moduleNumber) + "FF", ffVolts);
        driveClosedLoopController.setReference(
            velocityRadiansPerSecond,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVolts,
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        setAngle(new SwerveModuleState(0, rotation));
    }

    /**
     * Sets the angle of the module.
     * @param desiredState The desired state.
     */
    private void setAngle(SwerveModuleState desiredState) {
        // Reset it
        // if (moduleNumber == 1) {
        //     resetToAbsolute();
        // }

        // Stop the motor if the speed is less than 1%. Prevents Jittering
        if (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConfig.MAX_SPEED.in(MetersPerSecond) * 0.01)) {
            angleMotor.stopMotor();
            return;
        }

        // Set the angle using the PID controller
        Rotation2d angle = desiredState.angle;
        double degReference = angle.getDegrees();

        if (moduleNumber == 1) {
            debugAnglePidController.setSetpoint(degReference);

            double requestedVoltage = MathUtil.clamp(
                debugAnglePidController.calculate(
                    angleCANcoder.getPosition().getValue().in(Degrees) - angleOffset.getDegrees()
                ),
                -9,
                9
            );

            angleMotor.setVoltage(requestedVoltage);

            Logger.recordOutput("Swerve/Mod1/RequestedVoltage", requestedVoltage);
            Logger.recordOutput("Swerve/Mod1/PIDSetpoint", debugAnglePidController.getSetpoint());
        } else {
            angleClosedLoopController.setReference(degReference, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        // debug
        Logger.recordOutput("Swerve/Mod" + Integer.toString(moduleNumber) + "/Setpoint", degReference);
        Logger.recordOutput("Swerve/Mod" + Integer.toString(moduleNumber) + "/Current", relAngleEncoder.getPosition());
        Logger.recordOutput("Swerve/Mod" + Integer.toString(moduleNumber) + "/CANcoder", getCanCoder().getDegrees());
    }
}
