package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.GeometryUtils;

/** Swerve subsystem, responsible for controlling the swerve drive. */
public class Swerve extends SubsystemBase {

    /** The swerve odometry. This is used to determine the robot's position on the field. */
    public final SwerveDriveOdometry swerveOdometry;

    /**
     * The swerve modules. These are the four swerve modules on the robot. Each module has a drive
     * motor and a steering motor.
     */
    public final SwerveModule[] mSwerveMods;

    /** The gyro. This is used to determine the robot's heading. */
    public final Pigeon2 gyro;

    /**
     * A 2d representation of the field
     */
    private Field2d field = new Field2d();

    /** Creates a new Swerve subsystem. */
    public Swerve() {
        gyro = new Pigeon2(SwerveConstants.REV.pigeonID);
        // TODO: implement settings
        gyro.getConfigurator().apply(new Pigeon2Configuration());

        // Create swerve modules
        mSwerveMods = new SwerveModule[] {
            new SwerveMod(0, SwerveConstants.Swerve.Mod0.constants),
            new SwerveMod(1, SwerveConstants.Swerve.Mod1.constants),
            new SwerveMod(2, SwerveConstants.Swerve.Mod2.constants),
            new SwerveMod(3, SwerveConstants.Swerve.Mod3.constants),
        };

        swerveOdometry = new SwerveDriveOdometry(SwerveConfig.swerveKinematics, getYaw(), getModulePositions());
        zeroGyro();

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        // 2025 exclusive
        // RobotConfig config;
        // try{
        //     config = RobotConfig.fromGUISettings();
        // } catch (Exception e) {
        //     // Handle exception as needed
        //     e.printStackTrace();
        // }

        // Configure AutoBuilder
        // AutoBuilder.configureHolonomic(
        //     this::getPose,
        //     this::resetPose,
        //     this::getSpeeds,
        //     this::driveRobotRelative,
        //     SwerveConfig.pathFollowerConfig,
        //     () -> {
        //         // Boolean supplier that controls when the path will be mirrored for the red alliance
        //         // This will flip the path being followed to the red side of the field.
        //         // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //         Alliance alliance = DriverStation.getAlliance();
        //         if (alliance.isPresent()) {
        //             return alliance.get() == DriverStation.Alliance.Red;
        //         }
        //         return false;
        //     },
        //     this
        // );

        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

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
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S)
        );

        // Compute the twist (change in pose) required to reach the future pose
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);

        // Update the speeds based on the computed twist
        return new ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S
        );
    }

    /**
     * Drives the swerve modules based on the desired translation and rotation.
     *
     * @param translation The desired translation (x and y speeds).
     * @param rotation The desired rotation speed.
     * @param fieldRelative Whether the speeds are field-relative.
     * @param isOpenLoop Whether to use open-loop control.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // Determine the desired chassis speeds based on whether the control is field-relative
        ChassisSpeeds desiredChassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        // Correct the chassis speeds for robot dynamics
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

        // Convert the chassis speeds to swerve module states
        SwerveModuleState[] swerveModuleStates = SwerveConfig.swerveKinematics.toSwerveModuleStates(
            desiredChassisSpeeds
        );

        // Ensure the wheel speeds are within the allowable range
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConfig.maxSpeed);

        // Set the desired state for each swerve module
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    /**
     * Sets the desired states for the swerve modules. Used by SwerveControllerCommand in Auto.
     *
     * @param desiredStates The desired states for the swerve modules.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // System.out.println("setting module states: "+desiredStates[0]);

        // Ensure the wheel speeds are within the allowable range
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConfig.maxSpeed);

        // Set the desired state for each swerve module
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }

    /**
     * @return The current pose of the robot. This is determined by the swerve odometry.
     */
    public Pose2d getPose() {
        Pose2d p = swerveOdometry.getPoseMeters();
        return new Pose2d(-p.getX(), -p.getY(), p.getRotation());
    }

    /**
     * Resets the odometry of the robot.
     *
     * @param pose The new pose of the robot.
     */
    public void resetOdometry(Pose2d pose) {
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
        for (SwerveModule mod : mSwerveMods) {
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
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }

        return positions;
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
        swerveOdometry.update(getYaw(), getModulePositions());
    }

    /** Zeros the gyro, setting the angle to 0. */
    public void zeroGyro() {
        zeroGyro(0);
    }

    /**
     * @return The current yaw of the robot.
     */
    public Rotation2d getYaw() {
        // If the gyro is inverted, return the inverted yaw
        if (SwerveConfig.invertGyro) {
            return Rotation2d.fromDegrees(360 - gyro.getYaw().getValue());
        }

        // Otherwise, return the yaw as-is
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    /** Periodically updates the SmartDashboard with information about the swerve modules. */
    @Override
    public void periodic() {
        // Put the yaw on the SmartDashboard
        SmartDashboard.putNumber("yaw", gyro.getYaw().getValue());

        // Put the module information on the SmartDashboard
        for (SwerveModule mod : mSwerveMods) {
            /** The module name. Ex. "REV Mod 0" */
            String moduleName = String.format("REV Mod %d", mod.getModuleNumber());

            SmartDashboard.putNumber(moduleName + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber(moduleName + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber(moduleName + " Velocity", mod.getState().speedMetersPerSecond);
        }

        // Record module states
        Logger.recordOutput("SwerveModStates", getModuleStates());
        Logger.recordOutput("SwerveGyro", gyro.getRotation2d());

        Logger.recordOutput("SwervePose2d", swerveOdometry.getPoseMeters());
    }
}
