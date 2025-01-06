package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
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

    /** The angle motor. This motor is used to control the angle of the module. */
    private CANSparkMax mAngleMotor;

    /** The drive motor. This motor is used to control the speed of the module. */
    private CANSparkMax mDriveMotor;

    /** The angle encoder. This encoder is used to determine the angle of the module. */
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
        // Set the module number and angle offset.
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        // Create and configure the angle motor.
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        // Create and configure the drive motor.
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        // Create and configure the encoders.
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configEncoders();
    }

    /** Configures the encoders. */
    private void configEncoders() {
        // Reset the CANCoder to factory defaults and configure it.
        // angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getConfigurator().refresh(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(new SwerveConfig().canCoderConfig);

        // Assign the relative drive encoder and set the position to 0.
        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);

        // Set the position and velocity conversion factors based on the SwerveConfig
        relDriveEncoder.setPositionConversionFactor(SwerveConfig.driveRevToMeters);
        relDriveEncoder.setVelocityConversionFactor(SwerveConfig.driveRpmToMetersPerSecond);

        // Assign the relative angle encoder and configure it.
        relAngleEncoder = mAngleMotor.getEncoder();
        relAngleEncoder.setPositionConversionFactor(SwerveConfig.DegreesPerTurnRotation);
        relAngleEncoder.setVelocityConversionFactor(SwerveConfig.DegreesPerTurnRotation / 60); // in degrees/sec

        // Reset the module to absolute position.
        resetToAbsolute();

        // Write the configuration to flash memory.
        mDriveMotor.burnFlash();
        mAngleMotor.burnFlash();
    }

    /** Configures the angle motor. */
    private void configAngleMotor() {
        // Reset the angle motor to factory defaults
        mAngleMotor.restoreFactoryDefaults();

        // Configure the PID controller for the angle motor
        SparkPIDController controller = mAngleMotor.getPIDController();
        controller.setP(SwerveConfig.angleKP, 0);
        controller.setI(SwerveConfig.angleKI, 0);
        controller.setD(SwerveConfig.angleKD, 0);
        controller.setFF(SwerveConfig.angleKF, 0);
        controller.setOutputRange(-SwerveConfig.anglePower, SwerveConfig.anglePower);

        mAngleMotor.setSmartCurrentLimit(SwerveConfig.angleContinuousCurrentLimit);
        mAngleMotor.setInverted(SwerveConfig.angleMotorInvert);
        mAngleMotor.setIdleMode(SwerveConfig.angleIdleMode);
    }

    /** Configures the drive motor. */
    private void configDriveMotor() {
        // Reset the drive motor to factory defaults
        mDriveMotor.restoreFactoryDefaults();

        // Configure the PID controller for the drive motor
        SparkPIDController controller = mDriveMotor.getPIDController();
        controller.setP(SwerveConfig.driveKP, 0);
        controller.setI(SwerveConfig.driveKI, 0);
        controller.setD(SwerveConfig.driveKD, 0);
        controller.setFF(SwerveConfig.driveKF, 0);
        controller.setOutputRange(-SwerveConfig.drivePower, SwerveConfig.drivePower);

        mDriveMotor.setSmartCurrentLimit(SwerveConfig.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(SwerveConfig.driveMotorInvert);
        mDriveMotor.setIdleMode(SwerveConfig.driveIdleMode);
    }

    /**
     * Sets the desired state of the module.
     *
     * @param desiredState The desired state.
     * @param isOpenLoop Whether the module is in open loop.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // CTREModuleState functions for any motor type.
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);

        // Check for sensor faults.
        if (mDriveMotor.getFault(FaultID.kSensorFault)) {
            DriverStation.reportWarning("Sensor Fault on Drive Motor ID:" + mDriveMotor.getDeviceId(), false);
        }
        if (mAngleMotor.getFault(FaultID.kSensorFault)) {
            DriverStation.reportWarning("Sensor Fault on Angle Motor ID:" + mAngleMotor.getDeviceId(), false);
        }
    }

    /**
     * Sets the speed of the module.
     *
     * @param desiredState The desired state.
     * @param isOpenLoop Whether the module is in open loop.
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        // If the module is in open loop, set the speed directly.
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConfig.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }

        // Otherwise, set the speed using the PID controller.
        double velocity = desiredState.speedMetersPerSecond;

        SparkPIDController controller = mDriveMotor.getPIDController();
        controller.setReference(velocity, ControlType.kVelocity, 0);
    }

    /**
     * Sets the angle of the module.
     *
     * @param desiredState The desired state.
     */
    private void setAngle(SwerveModuleState desiredState) {
        // Stop the motor if the speed is less than 1%.
        // Prevents Jittering.
        if (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConfig.maxSpeed * 0.01)) {
            mAngleMotor.stopMotor();
            return;
        }

        // Set the angle using the PID controller.
        Rotation2d angle = desiredState.angle;
        double degReference = angle.getDegrees();

        SparkPIDController controller = mAngleMotor.getPIDController();
        controller.setReference(degReference, ControlType.kPosition, 0);
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
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue() * 360);
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

    /** Resets the module to absolute position. */
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
