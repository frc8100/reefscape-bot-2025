package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * Swerve configuration class. This class contains all the constants and configurations for the
 * swerve drive.
 */
public class SwerveConfig {

    /** How often the odometry is updated (in Hz). This is independent of the robot's. */
    public static final double odometryFrequency = 100;

    // Behavior of the motors when the robot is disabled/idling
    public static final SparkBaseConfig.IdleMode driveIdleMode = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode angleIdleMode = SparkBaseConfig.IdleMode.kBrake;

    // Percent output value limit for angle and drive motors
    public static final double drivePower = 0.7;
    public static final double anglePower = 0.9;

    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /**
     * Whether to use open loop control.
     * Default is `true`
     */
    public static final boolean isOpenLoop = true;

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(25.75);
    public static final double wheelBase = Units.inchesToMeters(21.50);
    public static final double wheelRadius = Units.inchesToMeters(2.0);
    public static final double wheelCircumference = wheelRadius * 2 * Math.PI;

    /** The module translations. No need to change. */
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
    };

    /* Module Gear Ratios */
    public static final double driveGearRatio = 6.75;
    public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);

    // Encoder setup
    // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderPositionFactor = (2 * Math.PI) / (driveGearRatio);
    // Rotor RPM -> Wheel Rad/Sec
    public static final double driveEncoderVelocityFactor = driveEncoderPositionFactor / 60;
    // the number of degrees that a single rotation of the turn motor turns the wheel.
    public static final double DegreesPerTurnRotation = 360 / angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = true;
    public static final boolean driveMotorInvert = false;

    /** Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

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

    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32);
    public static final double driveKV = (1.51);
    public static final double driveKA = (0.27);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.0;
    // Meters per Second^2
    public static final double maxAcceleration = 2.0;
    /** Radians per Second */
    public static final double maxAngularVelocity = 5.0;

    public static final double maxAngularAcceleration = 2.0;

    public static final double robotMassKg = 40.0;
    public static final double wheelCOF = 1.2;

    public static final Pose2d initialPose = new Pose2d(3, 3, new Rotation2d());
    public static final PathConstraints pathConstraints =
            new PathConstraints(maxSpeed, maxAcceleration, maxAngularVelocity, maxAngularAcceleration);

    // Simulator DC Motors
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    /**
     * Maplesim configuration for the swerve drive.
     */
    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withCustomModuleTranslations(moduleTranslations)
            .withRobotMass(Kilogram.of(robotMassKg))
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    driveGearbox,
                    turnGearbox,
                    // driveMotorReduction,
                    driveGearRatio,
                    // turnMotorReduction,
                    angleGearRatio,
                    Volts.of(0.1),
                    Volts.of(0.1),
                    Meters.of(wheelRadius),
                    KilogramSquareMeters.of(0.02),
                    wheelCOF));

    /**
     * @return The Pathplanner RobotConfig
     */
    // TODO: instead of loading from GUI, declare explicitly
    public static RobotConfig getRobotConfig() {
        // Load the RobotConfig from the GUI settings.
        RobotConfig config;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            config = null;
        }

        return config;
    }

    /**
     * @return The CANCoder configuration.
     */
    public static CANcoderConfiguration getCANcoderConfig() {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

        // The main way to configure the CANcoder is through MagnetSensorConfigs
        MagnetSensorConfigs magnetSenorConfig = new MagnetSensorConfigs()
                .withSensorDirection(
                        canCoderInvert
                                ? SensorDirectionValue.Clockwise_Positive
                                : SensorDirectionValue.CounterClockwise_Positive);

        canCoderConfig.withMagnetSensor(magnetSenorConfig);

        return canCoderConfig;
    }

    /**
     * @return The angle motor configuration.
     * Includes the relative encoder configuration.
     */
    public static SparkMaxConfig getAngleMotorConfig() {
        // Assign the relative angle encoder and configure it
        SparkMaxConfig angleConfig = new SparkMaxConfig();
        ;

        angleConfig
                .smartCurrentLimit(SwerveConfig.angleContinuousCurrentLimit)
                .inverted(SwerveConfig.angleMotorInvert)
                .idleMode(SwerveConfig.angleIdleMode);

        angleConfig
                .encoder
                .positionConversionFactor(SwerveConfig.DegreesPerTurnRotation)
                // The velocity conversion factor is in degrees/sec
                .velocityConversionFactor(SwerveConfig.DegreesPerTurnRotation / 60);

        // Configure the PID controller for the angle motor
        angleConfig
                .closedLoop
                .pidf(SwerveConfig.angleKP, SwerveConfig.angleKI, SwerveConfig.angleKD, SwerveConfig.angleKF)
                .outputRange(-SwerveConfig.anglePower, SwerveConfig.anglePower);

        angleConfig
                .signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / SwerveConfig.odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        return angleConfig;
    }

    /**
     * @return The drive motor configuration.
     * Includes the relative encoder configuration.
     */
    public static SparkMaxConfig getDriveMotorConfig() {
        // Get the config for the encoders
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        driveConfig
                .smartCurrentLimit(SwerveConfig.driveContinuousCurrentLimit)
                .inverted(SwerveConfig.driveMotorInvert)
                .idleMode(SwerveConfig.driveIdleMode);

        // Set the position and velocity conversion factors based on the SwerveConfig
        driveConfig
                .encoder
                .positionConversionFactor(SwerveConfig.driveEncoderPositionFactor)
                .velocityConversionFactor(SwerveConfig.driveEncoderVelocityFactor)
                // ! experimental
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        // Configure the PID controller for the drive motor
        driveConfig
                .closedLoop
                .pidf(SwerveConfig.driveKP, SwerveConfig.driveKI, SwerveConfig.driveKD, SwerveConfig.driveKF)
                .outputRange(-SwerveConfig.drivePower, SwerveConfig.drivePower);

        // ! experimental
        driveConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / SwerveConfig.odometryFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        return driveConfig;
    }
}
