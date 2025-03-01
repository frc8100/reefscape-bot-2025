package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * Swerve configuration class. This class contains all the constants and configurations for the
 * swerve drive.
 */
public class SwerveConfig {
    private SwerveConfig() {}

    /** How often the odometry is updated (in Hz). This is independent of the robot's. */
    public static final double ODOMETRY_FREQUENCY_HZ = 100;

    /**
     * The deadband for the sticks. This is the range of values that will be considered 0.
     */
    public static final double DRIVE_STICK_DEADBAND = 0.1;

    // Behavior of the motors when the robot is disabled/idling
    public static final SparkBaseConfig.IdleMode driveIdleMode = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode angleIdleMode = SparkBaseConfig.IdleMode.kBrake;

    // Percent output value limit for angle and drive motors
    public static final double MAX_DRIVE_POWER = 0.7;
    public static final double MAX_ANGLE_POWER = 0.9;

    public static final boolean IS_GYRO_INVERTED = false; // Always ensure Gyro is CCW+ CW-

    /**
     * Whether to use open loop control.
     * Default is `true`
     */
    public static final boolean IS_USING_OPEN_LOOP = true;

    // Drivetrain constants
    // TODO: precise track/base measurement
    public static final Distance TRACK_WIDTH = Inches.of(27);
    public static final Distance WHEEL_BASE = Inches.of(27);
    public static final Distance DRIVE_BASE_RADIUS =
            Meters.of(Math.hypot(TRACK_WIDTH.in(Meters) / 2.0, WHEEL_BASE.in(Meters) / 2.0));

    public static final Distance WHEEL_RADIUS = Inches.of(2.0);
    public static final Distance WHEEL_CIRCUMFERENCE = WHEEL_RADIUS.times(2 * Math.PI);

    /**
     * The locations of the swerve modules relative to the center of the robot. Used in {@link SwerveDriveKinematics}
     */
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
        new Translation2d(WHEEL_BASE.div(2), TRACK_WIDTH.div(2)),
        new Translation2d(WHEEL_BASE.div(2), TRACK_WIDTH.div(-2)),
        new Translation2d(WHEEL_BASE.div(-2), TRACK_WIDTH.div(2)),
        new Translation2d(WHEEL_BASE.div(-2), TRACK_WIDTH.div(-2)),
    };

    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double ANGLE_GEAR_RATIO = ((150.0 / 7.0) / 1.0);

    // Encoder setup
    /**
     * Factor to convert the drive encoder position from rotations to wheel radians.
     */
    public static final double DRIVE_ENCODER_POSITION_FACTOR = (2 * Math.PI) / (DRIVE_GEAR_RATIO);

    /**
     * Factor to convert the drive encoder velocity from RPM to wheel radians per second.
     */
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = DRIVE_ENCODER_POSITION_FACTOR / 60;

    /**
     * Factor to convert the angle encoder position from rotations to degrees
     */
    public static final double ANGLE_ENCODER_POSITION_FACTOR = 360 / ANGLE_GEAR_RATIO;

    /**
     * Factor to convert the angle encoder position from RPM to degrees/sec
     */
    public static final double ANGLE_ENCODER_VELOCITY_FACTOR = ANGLE_ENCODER_POSITION_FACTOR / 60;

    // Motor Inverts
    /**
     * Whether the angle motor is inverted.
     */
    public static final boolean IS_ANGLE_MOTOR_INVERTED = true;

    /**
     * Whether the drive motor is inverted.
     */
    public static final boolean IS_DRIVE_MOTOR_INVERTED = false;

    /** Whether the CANCoder is inverted. */
    public static final boolean IS_CANCODER_INVERTED = false;

    // Swerve Current Limiting
    public static final Current ANGLE_CONTINUOUS_CURRENT_LIMIT = Amps.of(20);
    public static final Current ANGLE_PEAK_CURRENT_LIMIT = Amps.of(40);
    public static final Time ANGLE_PEAK_CURRENT_DURATION = Seconds.of(0.1);
    public static final boolean IS_ANGLE_CURRENT_LIMIT_ACTIVE = true;

    public static final Current DRIVE_CONTINUOUS_CURRENT_LIMIT = Amps.of(35);
    public static final Current DRIVE_PEAK_CURRENT_LIMIT = Amps.of(60);
    public static final Time DRIVE_PEAK_CURRENT_DURATION = Seconds.of(0.1);
    public static final boolean IS_DRIVE_CURRENT_LIMIT_ACTIVE = true;

    // These values are used by the drive falcon to ramp in open loop and closed loop driving.
    // We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
    public static final double OPEN_LOOP_RAMP_VALUE = 0.25;
    public static final double CLOSED_LOOP_RAMP_VALUE = 0.0;

    // TODO: Change these to UPPER_SNAKE_CASE

    // Angle Motor PID Values
    public static final double angleKP = 0.05;
    public static final double angleKI = 0;
    public static final double angleKD = 0;
    public static final double angleKF = 0;

    public static final double angleSimKP = 8.0;
    public static final double angleSimKD = 0.0;

    // Drive Motor PID Values
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    // Drive Motor Characterization Values
    // Divide SYSID values by 12 to convert from volts to percent output for CTRE
    public static final double driveKS = (0.32);
    public static final double driveKV = (1.51);
    public static final double driveKA = (0.27);

    public static final double driveSimKP = 0.05;
    public static final double driveSimKD = 0.0;

    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Swerve Profiling Values
    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(5.0);
    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(2.0);
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(5.0);

    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(2.0);

    // Path Planner Values
    public static final Mass ROBOT_MASS = Kilogram.of(40.0);
    public static final double WHEEL_COF = 1.2;

    public static final Pose2d initialPose = new Pose2d(3, 3, new Rotation2d());
    public static final PathConstraints pathConstraints =
            new PathConstraints(MAX_SPEED, MAX_ACCELERATION, MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION);

    // Simulator DC Motors
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    /**
     * Maplesim configuration for the swerve drive.
     */
    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withCustomModuleTranslations(MODULE_TRANSLATIONS)
            .withRobotMass(ROBOT_MASS)
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    driveGearbox,
                    turnGearbox,
                    // driveMotorReduction,
                    DRIVE_GEAR_RATIO,
                    // turnMotorReduction,
                    ANGLE_GEAR_RATIO,
                    Volts.of(0.1),
                    Volts.of(0.1),
                    WHEEL_RADIUS,
                    KilogramSquareMeters.of(0.02),
                    WHEEL_COF));

    /**
     * @return The PathPlanner RobotConfig
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
                        IS_CANCODER_INVERTED
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

        angleConfig
                .smartCurrentLimit((int) SwerveConfig.ANGLE_CONTINUOUS_CURRENT_LIMIT.in(Amps))
                .inverted(SwerveConfig.IS_ANGLE_MOTOR_INVERTED)
                .idleMode(SwerveConfig.angleIdleMode);

        angleConfig
                .encoder
                .positionConversionFactor(SwerveConfig.ANGLE_ENCODER_POSITION_FACTOR)
                // The velocity conversion factor is in degrees/sec
                .velocityConversionFactor(SwerveConfig.ANGLE_ENCODER_VELOCITY_FACTOR);

        // Configure the PID controller for the angle motor
        angleConfig
                .closedLoop
                .pidf(SwerveConfig.angleKP, SwerveConfig.angleKI, SwerveConfig.angleKD, SwerveConfig.angleKF)
                .outputRange(-SwerveConfig.MAX_ANGLE_POWER, SwerveConfig.MAX_ANGLE_POWER);

        angleConfig
                .signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / SwerveConfig.ODOMETRY_FREQUENCY_HZ))
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
                .smartCurrentLimit((int) SwerveConfig.DRIVE_CONTINUOUS_CURRENT_LIMIT.in(Amps))
                .inverted(SwerveConfig.IS_DRIVE_MOTOR_INVERTED)
                .idleMode(SwerveConfig.driveIdleMode);

        // Set the position and velocity conversion factors based on the SwerveConfig
        driveConfig
                .encoder
                .positionConversionFactor(SwerveConfig.DRIVE_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(SwerveConfig.DRIVE_ENCODER_VELOCITY_FACTOR)
                // ! experimental
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        // Configure the PID controller for the drive motor
        driveConfig
                .closedLoop
                .pidf(SwerveConfig.driveKP, SwerveConfig.driveKI, SwerveConfig.driveKD, SwerveConfig.driveKF)
                .outputRange(-SwerveConfig.MAX_DRIVE_POWER, SwerveConfig.MAX_DRIVE_POWER);

        driveConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / SwerveConfig.ODOMETRY_FREQUENCY_HZ))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        return driveConfig;
    }
}
