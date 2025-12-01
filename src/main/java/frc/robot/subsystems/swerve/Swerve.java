package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.ControlConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.util.SwerveFeedForwards;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachineState;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Swerve subsystem, responsible for controlling the swerve drive. */
public class Swerve extends SubsystemBase implements SwerveDrive {

    /**
     * Lock for the odometry thread.
     */
    public static final Lock odometryLock = new ReentrantLock();

    private final TeleopSwerve teleopSwerve;

    public enum SwerveState {
        /**
         * The driver has full control over swerve. No autonomous actions are taken.
         */
        FULL_DRIVER_CONTROL,

        /**
         * The robot is driving to a target pose.
         * The driver has partial control over swerve and can nudge the robot in a direction.
         */

        /**
         * The robot is performing initial pathfinding to the target pose.
         */
        DRIVE_TO_POSE_PATHFINDING,

        /**
         * The robot is performing final alignment to the target pose using a simple PID controller.
         */
        DRIVE_TO_POSE_PID,

        /**
         * The robot has reached the target pose.
         */
        DRIVE_TO_POSE_AT_TARGET,

        /**
         * The robot is fully autonomous and following a pre-planned path.
         */
        FULL_AUTONOMOUS_PATH_FOLLOWING,
    }

    /**
     * The state machine for the swerve subsystem.
     * The payload is the target pose for the robot when in {@link SwerveState#DRIVE_TO_POSE}.
     */
    public final StateMachine<SwerveState, Supplier<Pose2d>> stateMachine = new StateMachine<
        SwerveState,
        Supplier<Pose2d>
    >(SwerveState.class, "Swerve")
        .withDefaultState(new StateMachineState<>(SwerveState.FULL_DRIVER_CONTROL, "Manual"))
        .withState(new StateMachineState<>(SwerveState.DRIVE_TO_POSE_PATHFINDING, "InitialPathfinding"))
        .withState(new StateMachineState<>(SwerveState.DRIVE_TO_POSE_PID, "PIDAlignment"))
        .withState(new StateMachineState<>(SwerveState.DRIVE_TO_POSE_AT_TARGET, "AtTarget"))
        .withState(new StateMachineState<>(SwerveState.FULL_AUTONOMOUS_PATH_FOLLOWING, "FollowPath"))
        .withReturnToDefaultStateOnDisable(true);

    private final SwerveFeedForwards swerveFeedForwards = new SwerveFeedForwards(this::isSimulation);

    private final SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
        SwerveConfig.getRobotConfig(),
        SwerveConfig.MAX_ANGULAR_VELOCITY_OF_SWERVE_MODULE
    );

    /**
     * Previous setpoints used for {@link #setpointGenerator}.
     */
    private SwerveSetpoint previousSetpoint;

    /**
     * The swerve modules. These are the four swerve modules on the robot. Each module has a drive
     * motor and a steering motor.
     */
    private final Module[] swerveModules = new Module[4];

    /**
     * The gyro. This is used to determine the robot's heading.
     */
    public final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    /**
     * Raw gyro rotation. Used for the pose estimator.
     */
    private Rotation2d rawGyroRotation = new Rotation2d();

    /**
     * Kinematics for the swerve drive. Used to convert between chassis speeds and module states.
     */
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConfig.MODULE_TRANSLATIONS);

    /**
     * The last stored position of the swerve modules for delta tracking.
     */
    private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
    };

    /**
     * A representation of the field for visualization (on Elastic).
     * Currently disabled to increase performance.
     * (use AdvantageScope for field visualization instead)
     */
    // protected Field2d field = new Field2d();

    private final SwerveDrivePoseEstimator poseEstimator;

    /**
     * The yaw offset for field-oriented driving.
     */
    private Rotation2d yawOffset = new Rotation2d();

    protected SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, state -> Logger.recordOutput("Swerve/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(voltage -> runCharacterization(voltage.in(Volt)), null, this)
    );

    /**
     * A trigger that syncs the motor encoders to the absolute encoders when the robot is still for a certain time.
     */
    private final Trigger syncMotorEncodersToAbsoluteEncoderTrigger = new Trigger(
        () -> getVelocityMagnitudeAsDouble() < SwerveConfig.STILL_MPS
    ).debounce(SwerveConfig.TIME_AFTER_STILL_SYNC_ENCODERS.in(Seconds));

    /** Creates a new Swerve subsystem. */
    public Swerve(GyroIO gyroIO, ModuleIO[] moduleIOs) {
        // Create the swerve modules
        for (int i = 0; i < 4; i++) {
            swerveModules[i] = new Module(moduleIOs[i], i);
        }

        this.gyroIO = gyroIO;

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            SwerveConfig.initialPose,
            SwerveConfig.stateStdDevs,
            SwerveConfig.visionStdDevs
        );

        zeroGyro(180);

        // Set up custom logging to add the current path to a field 2d widget
        // PathPlannerLogging.setLogActivePathCallback(poses -> field.getObject("path").setPoses(poses));
        // SmartDashboard.putData("Field", field);

        previousSetpoint = new SwerveSetpoint(
            getChassisSpeeds(),
            getModuleStates(),
            DriveFeedforwards.zeros(SwerveConfig.NUMBER_OF_SWERVE_MODULES)
        );

        teleopSwerve = new TeleopSwerve(
            this,
            // Switch between joystick and main drive controls depending on the mode
            ControlConstants.isUsingJoystickDrive
                ? ControlConstants.joystickDriveControls
                : ControlConstants.mainDriveControls,
            true
        );

        // Sync motor encoders to absolute encoders when the robot is still
        syncMotorEncodersToAbsoluteEncoderTrigger.onTrue(
            Commands.runOnce(() -> {
                for (Module module : swerveModules) {
                    module.syncMotorEncoderToAbsoluteEncoder();
                }

                // debug
                System.out.println("Swerve motor encoders synced");
            })
        );

        // Start odometry thread
        OdometryThread.getInstance().start();

        stop();
    }

    /**
     * @return The chassis speeds from a translation and rotation input, either field-relative or robot-relative.
     */
    public ChassisSpeeds getSpeedsFromTranslation(Translation2d translation, double rotation, boolean fieldRelative) {
        // Determine the desired chassis speeds based on whether the control is field-relative
        return fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getHeadingForFieldOriented()
            )
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        ChassisSpeeds desiredChassisSpeeds = getSpeedsFromTranslation(translation, rotation, fieldRelative);

        runVelocityChassisSpeeds(desiredChassisSpeeds);
    }

    @SuppressWarnings("unused")
    @Override
    public void runVelocityChassisSpeeds(ChassisSpeeds speed) {
        // Apply anti-tipping correction
        if (SwerveConfig.IS_ANTI_TIPPING_ENABLED && gyroInputs.isTipping) {
            ChassisSpeeds antiTippingSpeeds = gyroInputs.velocityAntiTipping;

            speed = antiTippingSpeeds;
        }

        // Convert the chassis speeds to swerve module states

        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speed, Constants.LOOP_PERIOD_SECONDS);

        SwerveModuleState[] setpointStates = previousSetpoint.moduleStates();
        DriveFeedforwards feedforwards = previousSetpoint.feedforwards();

        double[] feedforwardLinearForcesNewtons = feedforwards.linearForcesNewtons();

        // Log setpoints
        Logger.recordOutput("Swerve/States/Setpoints", setpointStates);
        Logger.recordOutput("Swerve/ChassisSpeeds/Setpoints", previousSetpoint.robotRelativeSpeeds());
        Logger.recordOutput("Swerve/ChassisSpeeds/SetpointsRaw", speed);

        Logger.recordOutput("Swerve/States/FeedforwardLinearForces", feedforwardLinearForcesNewtons);
        Logger.recordOutput("Swerve/States/FeedforwardTorqueCurrent", feedforwards.torqueCurrentsAmps());

        // Set the desired state for each swerve module
        setModuleStates(setpointStates, feedforwardLinearForcesNewtons);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates, double[] feedforwardLinearForcesNewtons) {
        // Set the desired state for each swerve module
        for (int i = 0; i < 4; i++) {
            Module mod = swerveModules[i];

            double driveFFVolts = swerveFeedForwards.getLinearForceFFVolts(
                desiredStates[mod.index].speedMetersPerSecond / SwerveConfig.WHEEL_RADIUS.in(Meters),
                feedforwardLinearForcesNewtons[mod.index]
            );

            mod.runSetpoint(desiredStates[mod.index], driveFFVolts);
        }
    }

    @Override
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].runCharacterization(output);
        }
    }

    @Override
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    @Override
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    @Override
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = swerveModules[i].getWheelRadiusCharacterizationPosition().in(Radians);
        }
        return values;
    }

    @Override
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += swerveModules[i].getFFCharacterizationVelocity().in(RadiansPerSecond) / 4.0;
        }
        return output;
    }

    public Optional<Double> getWheelSlippingCharacterization() {
        for (int i = 0; i < 4; i++) {
            // Check if the module is slipping by seeing if the velocity is nonzero
            if (swerveModules[i].getFFCharacterizationVelocity().in(RadiansPerSecond) < 0.175) continue;

            return Optional.of(swerveModules[i].getWheelSlippingCharacterization().in(Amps));
        }

        return Optional.empty();
    }

    @Override
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    @AutoLogOutput(key = "Swerve/States/Measured")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        // Get the state of each module
        for (int i = 0; i < 4; i++) {
            Module mod = swerveModules[i];
            states[mod.index] = mod.getState();
        }

        return states;
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            Module mod = swerveModules[i];
            positions[i] = mod.getPosition();
        }

        return positions;
    }

    @Override
    @AutoLogOutput(key = "Swerve/ChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    private final MutLinearVelocity cachedVelocityMagnitude = MetersPerSecond.mutable(0);

    /**
     * @return The magnitude of the robot's velocity. Calculated from {@link #getChassisSpeeds()}.
     */
    public LinearVelocity getVelocityMagnitude() {
        cachedVelocityMagnitude.mut_replace(getVelocityMagnitudeAsDouble(), MetersPerSecond);

        return cachedVelocityMagnitude;
    }

    /**
     * @return The velocity magnitude of the robot in meters per second.
     */
    @AutoLogOutput(key = "Swerve/ChassisSpeeds/Magnitude")
    public double getVelocityMagnitudeAsDouble() {
        ChassisSpeeds speeds = getChassisSpeeds();

        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    @Override
    public void zeroGyro(double deg) {
        yawOffset = getPose().getRotation().plus(Rotation2d.fromDegrees(deg));
    }

    @Override
    public Rotation2d getHeadingForFieldOriented() {
        return getPose().getRotation().minus(yawOffset);
    }

    @Override
    public void periodic() {
        // Prevents odometry updates while reading data
        odometryLock.lock();
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (Module module : swerveModules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            stop();
        }

        // Update odometry
        double[] sampleTimestamps = gyroInputs.odometryYawTimestamps; // All signals are sampled together
        int sampleCount = sampleTimestamps.length;

        for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas;

            if (!gyroInputs.connected) {
                moduleDeltas = new SwerveModulePosition[4];
            } else {
                // Not used when gyro is connected
                moduleDeltas = null;
            }

            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = swerveModules[moduleIndex].getOdometryPositions()[sampleIndex];

                // Only calculate module deltas if gyro is disconnected
                if (!gyroInputs.connected) {
                    moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle
                    );
                }

                // Update last positions
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[sampleIndex];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[sampleIndex], rawGyroRotation, modulePositions);
        }
    }
}
