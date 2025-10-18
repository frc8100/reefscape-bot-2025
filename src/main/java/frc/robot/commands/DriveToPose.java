package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.PoseUtil;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Uses PathPlanner to drive the robot to a specified pose on the field using path finding.
 */
public class DriveToPose extends Command {

    public enum DriveToPoseState {
        IDLE("Idle"),
        INITIAL_PATHFINDING("Initial pathfinding"),
        FINAL_ALIGNMENT("Final alignment"),
        AT_TARGET("At target");

        private final String description;

        public String getDescription() {
            return description;
        }

        DriveToPoseState(String description) {
            this.description = description;
        }
    }

    /**
     * The PID controller shared by all instances of this command.
     * Note: If multiple instances of this command are used simultaneously with different targets, this may cause issues.
     */
    // TODO: tune
    public static final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
        SwerveConfig.PP_ENDING_TRANSLATION_PID,
        SwerveConfig.PP_ROTATION_PID
    );

    // TODO: Move these to a constants file
    private static final Time debounceTime = Seconds.of(0.1);
    private static final Distance positionTolerance = Inches.of(1);
    private static final Angle angleTolerance = Degrees.of(3);
    private static final LinearVelocity speedTolerance = InchesPerSecond.of(2);

    private final SwerveDrive swerveSubsystem;

    /**
     * A supplier that provides the target pose to drive to.
     * Called once per command execution.
     */
    private final Supplier<Pose2d> targetPoseSupplier;

    /**
     * The current target pose. Updated every time the command is executed.
     * Use instead of calling the supplier multiple times.
     */
    private Pose2d targetPose;

    /**
     * A trigger that is true when the robot is at the target pose.
     * Has debounce.
     */
    private final Trigger atTarget;

    /**
     * Creates a new DriveToPose command.
     * @param swerveSubsystem - The swerve drive subsystem.
     * @param targetPoseSupplier - A supplier that provides the target pose to drive to. See {@link #targetPoseSupplier}.
     */
    public DriveToPose(SwerveDrive swerveSubsystem, Supplier<Pose2d> targetPoseSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;
        targetPose = targetPoseSupplier.get();

        addRequirements(swerveSubsystem);

        atTarget = new Trigger(() -> {
            Pose2d currentPose = swerveSubsystem.getPose();
            LinearVelocity currentSpeed = swerveSubsystem.getVelocityMagnitude();

            // Calculate errors
            return PoseUtil.isPosesAndVelocityNear(
                currentPose,
                targetPose,
                currentSpeed,
                MetersPerSecond.of(0),
                positionTolerance,
                angleTolerance,
                speedTolerance
            );
        }).debounce(debounceTime.in(Seconds));

        Logger.recordOutput("Swerve/DriveToPoseState", DriveToPoseState.IDLE.getDescription());
    }

    @Override
    public void initialize() {
        targetPose = targetPoseSupplier.get();

        // Reset controller state
        driveController.reset(swerveSubsystem.getPose(), new ChassisSpeeds());
    }

    @Override
    public void execute() {
        // Update state
        Logger.recordOutput("Swerve/DriveToPoseState", DriveToPoseState.FINAL_ALIGNMENT.getDescription());

        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = targetPoseSupplier.get();

        swerveSubsystem.runVelocityChassisSpeeds(
            driveController.calculateRobotRelativeSpeeds(swerveSubsystem.getPose(), goalState)
        );
    }

    @Override
    public boolean isFinished() {
        return atTarget.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Swerve/DriveToPoseState", DriveToPoseState.AT_TARGET.getDescription());

        if (interrupted) {
            swerveSubsystem.stop();
        }
    }
}
