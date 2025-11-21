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

// yall ts not tuff at allllll -Layla

package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.CANIdConnections;
import frc.robot.subsystems.CANIdConnections.SwerveModuleCanIDs;
import frc.robot.subsystems.swerve.SparkOdometryThread;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveModuleSpecificConstants;
import frc.robot.subsystems.swerve.SwerveModuleSpecificConstants.RobotSwerveModuleConstants;
import frc.util.CoupledYAMSSubsystemIO;
import frc.util.SparkUtil;
import frc.util.TunableValue;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

/**
 * Module IO implementation for Spark Max drive and angle motors with a CANcoder for absolute angle.
 */
public class ModuleIOSpark implements ModuleIO {

    public final int moduleNumber;

    /**
     * The dashboard key for this module. Used for logging.
     * Example: "Swerve/Module1"
     */
    private final String dashboardKey;

    /**
     * The angle offset. Used to zero the module to a specific angle.
     */
    private final Rotation2d angleOffset;

    /**
     * The angle motor. This motor is used to control the angle of the module.
     * Includes the integrated encoder {@link #relativeAngleEncoder} and an absolute CANcoder {@link #angleCANcoder}.
     */
    private final SparkMax angleMotor;

    private final CANcoder angleCANcoder;
    private final RelativeEncoder relativeAngleEncoder;
    private final SparkClosedLoopController angleClosedLoopController;

    /**
     * A PID controller that uses CANCoder angles exclusively, in degrees.
     */
    private final PIDController anglePidControllerUsingCANCoder;

    /**
     * The drive motor. This motor is used to control the speed of the module.
     * Includes an integrated encoder {@link #relativeDriveEncoder}.
     */
    private final SparkMax driveMotor;

    private final RelativeEncoder relativeDriveEncoder;
    private final SparkClosedLoopController driveClosedLoopController;

    // Status signal queues from CANcoder
    private final StatusSignal<Angle> turnAbsolutePosition;
    private final StatusSignal<Angle> turnRelativePosition;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Inputs (cached)
    private double driveFFVolts = 0.0;

    /**
     * Creates a new Swerve Module. Automatically gets CAN IDs and module constants based on module number.
     * @param moduleNumber - The module number.
     */
    public ModuleIOSpark(int moduleNumber) {
        this(
            moduleNumber,
            CANIdConnections.getModuleCANIdsFromIndex(moduleNumber),
            SwerveModuleSpecificConstants.getModuleConstantsFromIndex(moduleNumber)
        );
    }

    /**
     * Creates a new Swerve Module.
     * @param moduleNumber - The module number.
     * @param canIDs - The CAN IDs for the module.
     * @param moduleConstants - The module-specific constants.
     */
    public ModuleIOSpark(int moduleNumber, SwerveModuleCanIDs canIDs, RobotSwerveModuleConstants moduleConstants) {
        // Set the module number and angle offset
        this.moduleNumber = moduleNumber;
        this.dashboardKey = "Swerve/Module" + moduleNumber;
        this.angleOffset = moduleConstants.angleOffset();

        // Create and configure the angle motor
        angleMotor = new SparkMax(canIDs.angleMotorID(), MotorType.kBrushless);
        relativeAngleEncoder = angleMotor.getEncoder();
        angleClosedLoopController = angleMotor.getClosedLoopController();
        SparkUtil.configure(angleMotor, SwerveConfig.getAngleMotorConfig());

        // Create and configure the drive motor
        driveMotor = new SparkMax(canIDs.driveMotorID(), MotorType.kBrushless);
        relativeDriveEncoder = driveMotor.getEncoder();
        driveClosedLoopController = driveMotor.getClosedLoopController();
        SparkUtil.configure(driveMotor, SwerveConfig.getDriveMotorConfig());

        SparkUtil.tryUntilOk(driveMotor, 5, () -> relativeDriveEncoder.setPosition(0.0));

        // Create and configure the CANCoder
        angleCANcoder = new CANcoder(canIDs.canCoderID());
        angleCANcoder.getConfigurator().refresh(new CANcoderConfiguration());
        angleCANcoder.getConfigurator().apply(SwerveConfig.getCANcoderConfig());

        turnAbsolutePosition = angleCANcoder.getAbsolutePosition();
        turnAbsolutePosition.setUpdateFrequency(SwerveConfig.STATUS_SIGNAL_FREQUENCY_HZ);
        turnRelativePosition = angleCANcoder.getPosition();
        turnRelativePosition.setUpdateFrequency(SwerveConfig.STATUS_SIGNAL_FREQUENCY_HZ);
        angleCANcoder.optimizeBusUtilization();

        // Reset the module to absolute position
        syncMotorEncoderToAbsoluteEncoder();

        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance()
            .registerSignal(driveMotor, relativeDriveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(() -> getAngle().getRadians());

        TunableValue.addRefreshConfigConsumer(this::onRefresh);

        anglePidControllerUsingCANCoder = new PIDController(
            SwerveConfig.angleKP,
            SwerveConfig.angleKI,
            SwerveConfig.angleKD
        );
        anglePidControllerUsingCANCoder.enableContinuousInput(-180, 180);
        anglePidControllerUsingCANCoder.setSetpoint(getAngle().getDegrees());
    }

    /**
     * Refreshes the motor configurations from TunableValues.
     */
    private void onRefresh() {
        var newConfig = SwerveConfig.getDriveMotorConfig();
        newConfig.closedLoop.p(SwerveConfig.driveKPTunable.get()).d(SwerveConfig.driveKDTunable.get());

        driveMotor.configure(newConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        anglePidControllerUsingCANCoder.setPID(
            SwerveConfig.angleKPTunable.get(),
            SwerveConfig.angleKI,
            SwerveConfig.angleKDTunable.get()
        );

        // Print raw turn angle
        System.out.println(
            "Module " + moduleNumber + " Raw CANCoder Angle (radians): " + turnAbsolutePosition.getValue().in(Radians)
        );
    }

    /**
     * @return The angle of the module, gotten from the CANCoder. Includes angle offset.
     */
    private Rotation2d getAngle() {
        return new Rotation2d(turnAbsolutePosition.getValue().in(Radians)).minus(angleOffset);
    }

    /**
     * @return The relative angle of the module, gotten from the CANCoder. Includes angle offset.
     */
    private Rotation2d getRelativeAngle() {
        return new Rotation2d(turnRelativePosition.getValue().in(Radians)).minus(angleOffset);
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            relativeDriveEncoder.getVelocity() * SwerveConfig.WHEEL_RADIUS.in(Meters),
            getAngle()
        );
    }

    @Override
    public void syncMotorEncoderToAbsoluteEncoder() {
        double absolutePosition = getAngle().getDegrees();
        relativeAngleEncoder.setPosition(absolutePosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        inputs.driveMotorData = CoupledYAMSSubsystemIO.getDataFromSparkUnitless(driveMotor, relativeDriveEncoder);
        inputs.driveFFVolts = driveFFVolts;

        // Update turn inputs

        // Refresh CANCoder signals
        inputs.canCoderConnected = BaseStatusSignal.refreshAll(turnAbsolutePosition, turnRelativePosition).equals(
            StatusCode.OK
        );

        inputs.turnAbsolutePosition = getAngle();
        inputs.turnMotorData = CoupledYAMSSubsystemIO.getDataFromSparkUnitless(angleMotor, relativeAngleEncoder);

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream().map(Rotation2d::new).toArray(Rotation2d[]::new);
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
    public void setDesiredState(
        SwerveModuleState desiredState,
        Rotation2d currentRotation2d,
        double driveFeedforwardVoltage
    ) {
        setAngle(desiredState, currentRotation2d);
        setSpeed(desiredState, driveFeedforwardVoltage);
    }

    @Override
    public void setDriveVelocity(double speedMetersPerSecond, double driveFeedforwardVoltage) {
        setSpeed(new SwerveModuleState(speedMetersPerSecond, new Rotation2d()), driveFeedforwardVoltage);
    }

    /**
     * Sets the speed of the module.
     * @param desiredState The desired state.
     * @param isOpenLoop Whether the module is in open loop.
     */
    private void setSpeed(SwerveModuleState desiredState, double driveFeedforwardVoltage) {
        double velocityRadiansPerSecond = desiredState.speedMetersPerSecond / SwerveConfig.WHEEL_RADIUS.in(Meters);

        this.driveFFVolts = driveFeedforwardVoltage;

        driveClosedLoopController.setReference(
            velocityRadiansPerSecond,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            driveFeedforwardVoltage,
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        // TODO: update this
        setAngle(new SwerveModuleState(0, rotation), getAngle());
    }

    /**
     * Sets the angle of the module.
     * @param desiredState The desired state.
     */
    private void setAngle(SwerveModuleState desiredState, Rotation2d currentRotation2d) {
        // Stop the motor if the speed is less than 1%. Prevents Jittering
        if (
            Math.abs(currentRotation2d.minus(desiredState.angle).getDegrees()) < 0.5 &&
            Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConfig.MAX_SPEED.in(MetersPerSecond) * 0.01)
        ) {
            angleMotor.stopMotor();
            return;
        }

        // Set the angle using the PID controller
        Rotation2d angle = desiredState.angle;
        double degReference = angle.getDegrees();

        // TODO: Use spark encoder
        // angleClosedLoopController.setReference(degReference, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        anglePidControllerUsingCANCoder.setSetpoint(degReference);

        double requestedVoltage = MathUtil.clamp(
            anglePidControllerUsingCANCoder.calculate(getRelativeAngle().getDegrees()),
            -11,
            11
        );

        angleMotor.setVoltage(requestedVoltage);

        Logger.recordOutput(dashboardKey + "/RequestedVoltage", requestedVoltage);
        Logger.recordOutput(dashboardKey + "/PIDSetpoint", anglePidControllerUsingCANCoder.getSetpoint());
        Logger.recordOutput(dashboardKey + "/Current", getRelativeAngle().getDegrees());
    }
}
