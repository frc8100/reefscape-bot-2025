package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

/** a Swerve Modules using REV Robotics motor controllers and CTRE CANcoder absolute encoders. */
public class SwerveMod implements SwerveModule {

    /**
     * The module number identifier. Access using {@link #getModuleNumber} and {@link
     * #setModuleNumber}.
     */
    private int moduleNumber;

    /** The angle offset. Used to zero the module to a specific angle. */
    private Rotation2d angleOffset;

    /**
     * The angle motor. This motor is used to control the angle of the module.
     * Uses REV Robotics SparkMax.
     */
    private SparkMax mAngleMotor;

    /**
     * The drive motor. This motor is used to control the speed of the module.
     * Uses REV Robotics SparkMax.
     */
    private SparkMax mDriveMotor;

    /**
     * The angle encoder. This encoder is used to determine the angle of the module.
     * Uses CTRE Phoenix CANCoder.
     */
    private CANcoder angleEncoder;

    /**
     * The relative encoders. These encoders are used to determine the relative angle of the module.
     */
    private RelativeEncoder relAngleEncoder;

    /**
     * The relative drive encoder. This encoder is used to determine the relative position of the
     * module.
     */
    private RelativeEncoder relDriveEncoder;

    /**
     * Creates a new Swerve Module.
     *
     * @param moduleNumber The module number.
     * @param moduleConstants The module constants.
     */
    public SwerveMod(int moduleNumber, RevSwerveModuleConstants moduleConstants) {
        // Set the module number and angle offset
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        // Create and configure the angle motor
        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        // Create and configure the drive motor
        mDriveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        // Create and configure the CANCoder
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().refresh(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(new SwerveConfig().canCoderConfig);

        // Reset the module to absolute position
        resetToAbsolute();
    }

    /** Configures the angle motor. */
    private void configAngleMotor() {
        // Assign the relative angle encoder and configure it
        SparkMaxConfig angleConfig = new SparkMaxConfig();

        angleConfig.encoder
            .positionConversionFactor(SwerveConfig.DegreesPerTurnRotation)
            // The velocity conversion factor is in degrees/sec
            .velocityConversionFactor(SwerveConfig.DegreesPerTurnRotation / 60);

        // Reset the angle motor to factory defaults
        // mAngleMotor.restoreFactoryDefaults();

        // Configure the PID controller for the angle motor
        // Note: The 2025 version of SparkPIDController is SparkClosedLoopController
        // SparkClosedLoopController controller = mAngleMotor.getClosedLoopController();
        angleConfig.closedLoop
            .pidf(
                SwerveConfig.angleKP,
                SwerveConfig.angleKI,
                SwerveConfig.angleKD,
                SwerveConfig.angleKF
            )
            .outputRange(-SwerveConfig.anglePower, SwerveConfig.anglePower);
        // controller.setP(SwerveConfig.angleKP, 0);
        // controller.setI(SwerveConfig.angleKI, 0);
        // controller.setD(SwerveConfig.angleKD, 0);
        // controller.setFF(SwerveConfig.angleKF, 0);
        // controller.setOutputRange(-SwerveConfig.anglePower, SwerveConfig.anglePower);

        angleConfig
            .smartCurrentLimit(SwerveConfig.angleContinuousCurrentLimit)
            .inverted(SwerveConfig.angleMotorInvert)
            .idleMode(SwerveConfig.angleIdleMode);

        // mAngleMotor.setSmartCurrentLimit(SwerveConfig.angleContinuousCurrentLimit);
        // mAngleMotor.setInverted(SwerveConfig.angleMotorInvert);
        // mAngleMotor.setIdleMode(SwerveConfig.angleIdleMode);

        // ! IMPORTANT: New changes in 2025 may make this inaccurate
        mAngleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Configures the drive motor. */
    private void configDriveMotor() {
        // Get the config for the encoders
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        // Set the position and velocity conversion factors based on the SwerveConfig
        driveConfig.encoder
            .positionConversionFactor(SwerveConfig.driveRevToMeters)
            .velocityConversionFactor(SwerveConfig.driveRpmToMetersPerSecond);

        // Assign the relative drive encoder and set the position to 0
        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);

        // Configure the PID controller for the drive motor
        driveConfig.closedLoop
            .pidf(
                SwerveConfig.driveKP,
                SwerveConfig.driveKI,
                SwerveConfig.driveKD,
                SwerveConfig.driveKF
            )
            .outputRange(-SwerveConfig.drivePower, SwerveConfig.drivePower);

        driveConfig
            .smartCurrentLimit(SwerveConfig.driveContinuousCurrentLimit)
            .inverted(SwerveConfig.driveMotorInvert)
            .idleMode(SwerveConfig.driveIdleMode);

        // ! IMPORTANT: New changes in 2025 may make this inaccurate
        mDriveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the desired state of the module.
     *
     * @param desiredState The desired state.
     * @param isOpenLoop Whether the module is in open loop.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // CTREModuleState functions for any motor type
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);

        // TODO: Check for sensor faults
        // if (mDriveMotor.getFaults(Faults.kSensorFault)) {
        //     DriverStation.reportWarning("Sensor Fault on Drive Motor ID:" + mDriveMotor.getDeviceId(), false);
        // }
        // if (mAngleMotor.getFault(FaultID.kSensorFault)) {
        //     DriverStation.reportWarning("Sensor Fault on Angle Motor ID:" + mAngleMotor.getDeviceId(), false);
        // }
    }

    /**
     * Sets the speed of the module.
     *
     * @param desiredState The desired state.
     * @param isOpenLoop Whether the module is in open loop.
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        // If the module is in open loop, set the speed directly
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConfig.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }

        // Otherwise, set the speed using the PID controller
        double velocity = desiredState.speedMetersPerSecond;

        SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();
        controller.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    /**
     * Sets the angle of the module.
     *
     * @param desiredState The desired state.
     */
    private void setAngle(SwerveModuleState desiredState) {
        // Stop the motor if the speed is less than 1%
        // Prevents Jittering
        if (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConfig.maxSpeed * 0.01)) {
            mAngleMotor.stopMotor();
            return;
        }

        // Set the angle using the PID controller
        Rotation2d angle = desiredState.angle;
        double degReference = angle.getDegrees();

        // SparkPIDController controller = mAngleMotor.getPIDController();
        SparkClosedLoopController controller = mAngleMotor.getClosedLoopController();
        controller.setReference(degReference, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    /**
     * @return The angle of the module.
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
    }

    /**
     * @return The CANCoder angle of the module.
     */
    public Rotation2d getCanCoder() {
        // return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue() * 360);
        // ! IMPORTANT: New changes in 2025 may make this inaccurate
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    /**
     * @return The module number.
     */
    public int getModuleNumber() {
        return moduleNumber;
    }

    /**
     * Sets the module number.
     *
     * @param moduleNumber The module number.
     */
    public void setModuleNumber(int moduleNumber) {
        this.moduleNumber = moduleNumber;
    }

    /**
     * Resets the module to absolute position.
     * This is used to zero the module to a specific angle.
     * It is also called when the robot is enabled to reset the module to the absolute position.
     */
    private void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        relAngleEncoder.setPosition(absolutePosition);
    }

    /**
     * @return The state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(relDriveEncoder.getVelocity(), getAngle());
    }

    /**
     * @return The position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(relDriveEncoder.getPosition(), getAngle());
    }
}
