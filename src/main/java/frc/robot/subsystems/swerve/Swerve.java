package frc.robot.subsystems.swerve;

import com.pathplanner.lib.util.PathPlannerLogging;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.GeometryUtils;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.path.SwervePathFollow;
import java.util.ArrayList;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Swerve subsystem, responsible for controlling the swerve drive. */
public class Swerve extends SubsystemBase implements SwerveDrive {
    /** Lock for the odometry thread. */
    static final Lock odometryLock = new ReentrantLock();

    /**
     * The swerve modules. These are the four swerve modules on the robot. Each module has a drive
     * motor and a steering motor.
     */
    private final Module[] swerveModules = new Module[4];

    /** The gyro. This is used to determine the robot's heading. */
    public final GyroIO gyroIO;

    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    /** Raw gryo rotation. Used for the pose estimator. */
    private Rotation2d rawGyroRotation = new Rotation2d();

    /**
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve
     */
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConfig.moduleTranslations);

    /** The last stored position of the swerve modules for delta tracking */
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    /** A 2d representation of the field */
    protected Field2d field = new Field2d();

    /** The pathfinding */
    public final SwervePathFollow pathfinding = new SwervePathFollow(this);

    /** The routines to run */
    private ArrayList<RoutineWithCondition> routines = new ArrayList<>();

    /** The callback to reset the simulation pose */
    // private final Consumer<Pose2d> resetSimulationPoseCallBack;

    /**
     * Pose estimator. This is the same as odometry but includes vision input to correct for
     * drifting.
     */
    private SwerveDrivePoseEstimator poseEstimator;

    /** Creates a new Swerve subsystem. */
    public Swerve(GyroIO gyroIO, ModuleIO[] moduleIOs) {
        // Create the swerve modules
        for (int i = 0; i < 4; i++) {
            swerveModules[i] = new Module(moduleIOs[i], i);
        }

        this.gyroIO = gyroIO;

        // Start odometry thread
        SparkOdometryThread.getInstance().start();

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                rawGyroRotation,
                lastModulePositions,
                new Pose2d(),
                Constants.PoseEstimator.stateStdDevs,
                Constants.PoseEstimator.VisionStdDevs);

        zeroGyro();

        configurePathPlannerAutoBuilder();

        // TODO: Configure SysId
        // sysId =
        //         new SysIdRoutine(
        //                 new SysIdRoutine.Config(
        //                         null,
        //                         null,
        //                         null,
        //                         (state) -> Logger.recordOutput("Drive/SysIdState",
        // state.toString())),
        //                 new SysIdRoutine.Mechanism(
        //                         (voltage) -> runCharacterization(voltage.in(Volts)), null,
        // this));

        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback(
                (poses) -> field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", field);
    }

    @Override
    public SwervePathFollow getPathfindingInstance() {
        return pathfinding;
    }

    /**
     * Corrects for the dynamics of the robot. This is used to ensure that the robot drives as
     * expected.
     *
     * @param originalSpeeds The original chassis speeds.
     * @return The corrected chassis speeds.
     */
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        // Loop time in seconds
        final double LOOP_TIME_S = 0.02;

        // Calculate the future robot pose based on the original speeds and loop time
        Pose2d futureRobotPose = new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));

        // Compute the twist (change in pose) required to reach the future pose
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);

        // Update the speeds based on the computed twist
        return new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S, twistForPose.dy / LOOP_TIME_S, twistForPose.dtheta / LOOP_TIME_S);
    }

    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        // Determine the desired chassis speeds based on whether the control is field-relative
        ChassisSpeeds desiredChassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), translation.getY(), rotation, gyroIO.getGyroHeading())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        // Correct the chassis speeds for robot dynamics
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

        runVelocityChassisSpeeds(desiredChassisSpeeds);
    }

    @Override
    public void runVelocityChassisSpeeds(ChassisSpeeds speed) {
        // Convert the chassis speeds to swerve module states
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speed, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);

        // Ensure the wheel speeds are within the allowable range
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConfig.maxSpeed);

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Set the desired state for each swerve module
        // for (Module mod : swerveModules) {
        for (int i = 0; i < 4; i++) {
            Module mod = swerveModules[i];
            mod.runSetpoint(setpointStates[mod.index]);
        }
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Ensure the wheel speeds are within the allowable range
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConfig.maxSpeed);

        // Set the desired state for each swerve module
        // for (Module mod : swerveModules) {
        for (int i = 0; i < 4; i++) {
            Module mod = swerveModules[i];
            // mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
            mod.runSetpoint(desiredStates[mod.index]);
        }
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
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /**
     * Resets the odometry of the robot.
     *
     * @param pose The new pose of the robot.
     */
    private void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(new Rotation2d(), getModulePositions(), pose);
        zeroGyro(pose.getRotation().getDegrees());
    }

    @Override
    @AutoLogOutput(key = "SwerveStates/Measured")
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
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    @Override
    public void zeroGyro(double deg) {
        gyroIO.zeroGyro(deg);

        poseEstimator.update(gyroIO.getGyroHeading(), getModulePositions());
    }

    @Override
    public Rotation2d getGyroHeading() {
        return gyroIO.getGyroHeading();
    }

    @Override
    public void registerRoutineWhileCondition(RoutineWithCondition routine) {
        routines.add(routine);
    }

    /** Periodically updates the SmartDashboard with information about the swerve modules. */
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

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = swerveModules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = swerveModules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

            // Check for routines
            // for (RoutineWithCondition routine : routines) {
            //     if (routine.condition.getAsBoolean()) {
            //         routine.routine.get().schedule();
            //     }
            // }
        }
    }
}
