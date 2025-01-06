package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants;

/**
 * Swerve configuration class. This class contains all the constants and configurations for the
 * swerve drive.
 */
public class SwerveConfig {

    /** The CANCoder configuration. */
    public final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    //
    public static final IdleMode driveIdleMode = IdleMode.kBrake;
    public static final IdleMode angleIdleMode = IdleMode.kBrake;
    public static final double drivePower = 1;
    public static final double anglePower = .9;

    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSNeoSwerveConstants chosenModule = COTSNeoSwerveConstants.SDSMK4i(
        COTSNeoSwerveConstants.driveGearRatios.SDSMK4i_L2
    );

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(25.75);
    public static final double wheelBase = Units.inchesToMeters(21.50);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    );

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    // encoder setup
    // meters per rotation
    public static final double driveRevToMeters = wheelCircumference / (driveGearRatio);
    public static final double driveRpmToMetersPerSecond = driveRevToMeters / 60;
    // the number of degrees that a single rotation of the turn motor turns the wheel.
    public static final double DegreesPerTurnRotation = 360 / angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.05;
    public static final double angleKI = 0;
    public static final double angleKD = 0;
    public static final double angleKF = 0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32);
    public static final double driveKV = (1.51);
    public static final double driveKA = (0.27);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.0;
    /** Radians per Second */
    public static final double maxAngularVelocity = 5.0; // max 10 or.....

    /**
     * pplib config, temporary
     */
    // TODO: Update to 2025
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(5.0, 0, 0), // Translation constants 
        new PIDConstants(5.0, 0, 0), // Rotation constants 
        maxSpeed,
        // flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module)
        // Placeholder
        0.1,
        new ReplanningConfig()
    );

    /** Configures the swerve config */
    public SwerveConfig() {
        // Set up the CANCoder configuration
        MagnetSensorConfigs magnetSenorConfig = new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withSensorDirection(
                canCoderInvert
                    ? SensorDirectionValue.Clockwise_Positive
                    : SensorDirectionValue.CounterClockwise_Positive
            );

        canCoderConfig.withMagnetSensor(magnetSenorConfig);
        // TODO: Update to phoenix 6 (missing config, SensorDirection from boolean to enum)
        // canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive
        // canCoderConfig.MagnetSensor.SensorDirection = canCoderInvert ?
        // SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        // canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0_To360;
        // canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        // canCoderConfig.MagnetSensor.initializationStrategy =
        // SensorInitializationStrategy.BootToAbsolutePosition;
        // canCoderConfig.MagnetSensor.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
