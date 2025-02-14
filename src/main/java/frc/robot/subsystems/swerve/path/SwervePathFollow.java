package frc.robot.subsystems.swerve.path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Pathfinding class for the swerve drive. This class will be used to generate paths for the robot to follow.
 */
public class SwervePathFollow {
    /**
     * The maximum duration of a path in seconds.
     * If the path takes longer than this, it will be cancelled.
     */
    // TODO: Move to constants
    public static final double maxPathDurationSeconds = 10.0;

    /**
     * Locations of the robot to interact with the field elements.
     * Note: not flipped for the blue side.
     */
    public static class FieldLocations {
        public static final Pose2d coralStation1 = new Pose2d(1.1, 1.1, Rotation2d.fromDegrees(55));
        public static final Pose2d coralStation2 = new Pose2d(1.1, 6.8, Rotation2d.fromDegrees(-55));

        public static final Pose2d reef1 = new Pose2d(3.6, 5.5, Rotation2d.fromDegrees(-60));
    }

    @FunctionalInterface
    public interface CommandSupplier {
        /**
         * @param isOuterCommandRunning - A supplier to check if the outer command is running. If false, the command should be cancelled.
         * @return The command to run.
         */
        Command get(BooleanSupplier isOuterCommandRunning);
    }

    // Suppliers
    // private Supplier<Pose2d> robotPositionSupplier;
    // private Supplier<Rotation2d> robotHeadingSupplier;
    // private Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private SwerveDrive swerveSubsystem;

    /**
     * The current path the robot is following.
     */
    // private Optional<Command> currentCommand = Optional.empty();
    /**
     * Whether or not the inner command should be running.
     * This is used to cancel the inner command when the outer command is cancelled.
     */
    private boolean shouldInnerCommandbeRunning = false;

    /**
     * The consumer to cancel the outer command when the inner command is cancelled/finished.
     */
    // private Optional<Consumer<?>> cancelOuterCommand = Optional.empty();
    /**
     * Whether or not the outer command should be running.
     * This is used to cancel the outer command when the inner command is cancelled.
     */
    private boolean shouldOuterCommandBeRunning = false;

    /**
     * Creates a new Pathfinding class given the robot's position and heading suppliers.
     */
    // public SwervePathFollow(Supplier<Pose2d> robotPositionSupplier, Supplier<Rotation2d> robotHeadingSupplier) {
    public SwervePathFollow(SwerveDrive swerveSubsystem) {
        // this.robotPositionSupplier = robotPositionSupplier;
        // this.robotHeadingSupplier = robotHeadingSupplier;
        this.swerveSubsystem = swerveSubsystem;
    }

    /**
     * Generates a path to the given pose. Uses the current robot position and heading.
     */
    private PathPlannerPath generatePath(Pose2d poseToGoTo) {
        // Get the current robot position and heading
        Pose2d currentPose = swerveSubsystem.getPose();
        Rotation2d currentRotation = currentPose.getRotation();

        // Create a path to the target pose
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, poseToGoTo);

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                SwerveConfig.pathConstraints,
                new IdealStartingState(
                        // LinearVelocity.ofBaseUnits(
                        //         Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                        // Units.MetersPerSecond),
                        LinearVelocity.ofBaseUnits(0, Units.MetersPerSecond), currentRotation),
                new GoalEndState(0, poseToGoTo.getRotation()));

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        return path;
    }

    /**
     * Generates a command to go to the given pose.
     * Note: once called, it will not update the path if the robot moves.
     * Therefore, it is recommended to use {@link #generateDynamicPathFollow(SwerveDrive, Pose2d)} instead.
     */
    public Command generateGoToPoseCommand(Pose2d poseToGoTo) {
        PathPlannerPath path = generatePath(poseToGoTo);

        // Create a command to follow the path
        Command followPathCommand;
        try {
            // This sometimes errors if the robot is at the same position as the target pose and is rotated
            followPathCommand = AutoBuilder.followPath(path);
        } catch (Exception e) {
            // Return an empty command if the path is invalid
            return Commands.runOnce(() -> {}, swerveSubsystem);
        }

        followPathCommand.addRequirements(swerveSubsystem);

        return followPathCommand;
    }
}
