package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.GeometryUtils;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Swerve subsystem, responsible for controlling the swerve drive. */
public class Swerve extends SubsystemBase {

    /**
     * The swerve modules. These are the four swerve modules on the robot. Each module has a drive
     * motor and a steering motor.
     */
    public final SwerveModule[] swerveModules = new SwerveModule[] {
        new SwerveMod(0, SwerveConstants.Swerve.Mod0.constants),
        new SwerveMod(1, SwerveConstants.Swerve.Mod1.constants),
        new SwerveMod(2, SwerveConstants.Swerve.Mod2.constants),
        new SwerveMod(3, SwerveConstants.Swerve.Mod3.constants),
    };

    /** The gyro. This is used to determine the robot's heading. */
    public final Pigeon2 gyro = new Pigeon2(SwerveConstants.REV.pigeonID);

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve
     */
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConfig.moduleTranslations);

    /** The last stored position of the swerve modules for delta tracking */
    // TODO: Implement this
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    /** A 2d representation of the field */
    private Field2d field = new Field2d();

    /** Raw gryo rotation. Used for the pose estimator. */
    // TODO: Implement this
    private Rotation2d rawGyroRotation = new Rotation2d();

    /**
     * Pose estimator. This is the same as odometry but includes vision input to correct for
     * drifting.
     */
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            // rawGyroRotation,
            getGyroHeading(),
            // lastModulePositions,
            getModulePositions(),
            new Pose2d(),
            Constants.PoseEstimator.stateStdDevs,
            Constants.PoseEstimator.VisionStdDevs);

    /** The swerve odometry. This is used to determine the robot's position on the field. */
    public final SwerveDriveOdometry swerveOdometry;

    /** Creates a new Swerve subsystem. */
    public Swerve() {
        // TODO: implement settings
        gyro.getConfigurator().apply(new Pigeon2Configuration());

        swerveOdometry = new SwerveDriveOdometry(kinematics, getGyroHeading(), getModulePositions());

        zeroGyro();

        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocityChassisSpeeds,
                new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                SwerveConstants.getRobotConfig(),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStar());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

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
        // Logger.recordOutput("Field2d", field);
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

    /**
     * Drives the swerve modules based on the desired translation and rotation.
     *
     * @param translation The desired translation (x and y speeds).
     * @param rotation The desired rotation speed.
     * @param fieldRelative Whether the speeds are field-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        // Determine the desired chassis speeds based on whether the control is field-relative
        ChassisSpeeds desiredChassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), translation.getY(), rotation, getGyroHeading())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        // Correct the chassis speeds for robot dynamics
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

        runVelocityChassisSpeeds(desiredChassisSpeeds);
    }

    /**
     * Drives the swerve modules given a provided chassis speeds.
     * @param speed The desired chasssis speeds
     */
    public void runVelocityChassisSpeeds(ChassisSpeeds speed) {
        // TODO: debug
        speed = speed.times(SwerveConstants.debugSpeedMultiplier);

        // Convert the chassis speeds to swerve module states
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speed, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);

        // Ensure the wheel speeds are within the allowable range
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConfig.maxSpeed);

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Set the desired state for each swerve module
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(setpointStates[mod.getModuleNumber()], SwerveConstants.isOpenLoop);
        }
    }

    /**
     * Sets the desired states for the swerve modules. Used by SwerveControllerCommand in Auto.
     *
     * @param desiredStates The desired states for the swerve modules.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Ensure the wheel speeds are within the allowable range
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConfig.maxSpeed);

        // Set the desired state for each swerve module
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }

    /**
     * @return The current pose of the robot. This is determined by the swerve odometry.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        // return poseEstimator.getEstimatedPosition();
        // TODO: Temporary
        Logger.recordOutput("PoseEstimator/Robot", poseEstimator.getEstimatedPosition());
        Pose2d p = swerveOdometry.getPoseMeters();
        return new Pose2d(-p.getX(), -p.getY(), p.getRotation());
    }

    /**
     * @return the current odometry rotation.
     */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
        swerveOdometry.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /**
     * Resets the odometry of the robot.
     *
     * @param pose The new pose of the robot.
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(new Rotation2d(), getModulePositions(), pose);
        swerveOdometry.resetPosition(new Rotation2d(), getModulePositions(), pose);
        zeroGyro(pose.getRotation().getDegrees());
    }

    /**
     * @return The current module states.
     */
    public SwerveModuleState[] getModuleStates() {
        // Create an array to hold the module states
        SwerveModuleState[] states = new SwerveModuleState[4];

        // Get the state of each module
        for (SwerveModule mod : swerveModules) {
            states[mod.getModuleNumber()] = mod.getState();
        }

        return states;
    }

    /**
     * @return The current module positions.
     */
    public SwerveModulePosition[] getModulePositions() {
        // Create an array to hold the module positions
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        // Get the position of each module
        for (SwerveModule mod : swerveModules) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }

        return positions;
    }

    /**
     * @return the measured chassis speeds of the robot.
     */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Zeros the gyro.
     *
     * @param deg The angle to zero the gyro to.
     */
    public void zeroGyro(double deg) {
        // Invert the gyro if necessary
        if (SwerveConfig.invertGyro) {
            deg = -deg;
        }

        // Zero the gyro and update the odometry
        gyro.setYaw(deg);
        swerveOdometry.update(getGyroHeading(), getModulePositions());
        poseEstimator.update(getGyroHeading(), getModulePositions());
    }

    /** Zeros the gyro, setting the angle to 0. */
    public void zeroGyro() {
        zeroGyro(0);
    }

    /**
     * @return The current gyro yaw of the robot.
     */
    @AutoLogOutput(key = "Gyro/Yaw")
    public Rotation2d getYaw() {
        // If the gyro is inverted, return the inverted yaw
        if (SwerveConfig.invertGyro) {
            return Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble());
        }

        // Otherwise, return the yaw as-is
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    /**
     * @return The current gyro heading of the robot.
     */
    @AutoLogOutput(key = "Gyro/Heading")
    public Rotation2d getGyroHeading() {
        // If the gyro is inverted, return the inverted yaw
        if (SwerveConfig.invertGyro) {
            return gyro.getRotation2d().rotateBy(new Rotation2d(180));
        }

        // Otherwise, return the yaw as-is
        return gyro.getRotation2d();
    }

    /** Stops the drive. */
    public void stop() {
        runVelocityChassisSpeeds(new ChassisSpeeds());
    }

    /** Periodically updates the SmartDashboard with information about the swerve modules. */
    @Override
    public void periodic() {
        // Update Odometry
        swerveOdometry.update(getGyroHeading(), getModulePositions());
        poseEstimator.update(getGyroHeading(), getModulePositions());

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            stop();
        }

        // Put the yaw on the SmartDashboard
        // SmartDashboard.putNumber("yaw", gyro.getYaw().getValueAsDouble());

        double[] canCoderOutputs = new double[4];
        double[] integratedOutputs = new double[4];
        double[] velocityOutputs = new double[4];

        // Put the module information on the SmartDashboard
        for (SwerveModule mod : swerveModules) {
            int modId = mod.getModuleNumber();

            /** The module name. Ex. "REV Mod 0" */
            String moduleName = String.format("REV Mod %d ", modId);
            // String akKey = String.format("RevMod%d/", modId);

            double canCoder = mod.getCanCoder().getDegrees();
            double integrated = mod.getPosition().angle.getDegrees();
            double velocity = mod.getState().speedMetersPerSecond;

            SmartDashboard.putNumber(moduleName + "Cancoder", canCoder);
            canCoderOutputs[modId] = canCoder;
            // Logger.recordOutput(akKey + "Cancoder", canCoder);

            SmartDashboard.putNumber(moduleName + "Integrated", integrated);
            integratedOutputs[modId] = integrated;
            // Logger.recordOutput(akKey + "Integrated", integrated);

            SmartDashboard.putNumber(moduleName + "Velocity", velocity);
            velocityOutputs[modId] = velocity;
            // Logger.recordOutput(akKey + "Velocity", velocity);
        }

        // Record module states
        Logger.recordOutput("SwerveModStates", getModuleStates());
        Logger.recordOutput("SwerveGyro", gyro.getRotation2d());

        Logger.recordOutput("SwerveModuleCancoder", canCoderOutputs);
        Logger.recordOutput("SwerveModuleIntegrated", integratedOutputs);
        Logger.recordOutput("SwerveModuleVelocity", velocityOutputs);

        Logger.recordOutput("SwervePose2dOdometry", swerveOdometry.getPoseMeters());
        // Logger.recordOutput("SwervePose2dOdometry", poseEstimator.getEstimatedPosition());
    }
}
