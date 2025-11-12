package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.PoseUtil;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Uses a PID controller to drive the robot to a specified pose. Does not use path finding/obstacle avoidance.
 */
public class DriveToPose {

    /**
     * The PID controller shared by all instances of this command.
     * Note: If multiple instances of this command are used simultaneously with different targets, this may cause issues.
     */
    public static final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
        SwerveConfig.PP_ENDING_TRANSLATION_PID,
        SwerveConfig.PP_ROTATION_PID
    );

    // Tolerances for being "at" the target pose
    private static final Distance positionTolerance = Inches.of(1.25);
    private static final Angle angleTolerance = Degrees.of(3);
    private static final LinearVelocity speedTolerance = InchesPerSecond.of(4);

    private final SwerveDrive swerveSubsystem;

    /**
     * A supplier that provides the target pose to drive to.
     * Called once per command execution.
     */
    private Supplier<Pose2d> targetPoseSupplier;

    /**
     * The current target pose. Updated every time the command is executed.
     * Use instead of calling the supplier multiple times.
     */
    private Pose2d targetPose = new Pose2d();

    /**
     * A trigger that is true when the robot is at the target pose.
     * Has debounce.
     */
    public final BooleanSupplier atTarget;

    public final BooleanSupplier debugPoseTranslationsNear;
    public final BooleanSupplier debugPoseRotationsNear;
    public final BooleanSupplier debugVelocityNear;

    /**
     * Whether the robot can switch to final alignment mode.
     */
    public final BooleanSupplier canSwitchToFinalAlignment;

    /**
     * Creates a new DriveToPose command.
     * @param swerveSubsystem - The swerve drive subsystem.
     * @param targetPoseSupplier - A supplier that provides the target pose to drive to. See {@link #targetPoseSupplier}.
     */
    public DriveToPose(SwerveDrive swerveSubsystem, Supplier<Pose2d> targetPoseSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;
        targetPose = targetPoseSupplier.get();

        // addRequirements(swerveSubsystem);

        atTarget = () ->
            PoseUtil.isPosesAndVelocityNear(
                swerveSubsystem.getPose(),
                this.targetPoseSupplier.get(),
                swerveSubsystem.getVelocityMagnitude(),
                MetersPerSecond.of(0),
                positionTolerance,
                angleTolerance,
                speedTolerance
            );

        canSwitchToFinalAlignment = () ->
            PoseUtil.isNear(
                swerveSubsystem.getPose(),
                this.targetPoseSupplier.get(),
                // Start final alignment when within this distance plus a bit based on current speed
                Meters.of(0.4 + swerveSubsystem.getVelocityMagnitude().in(MetersPerSecond) * 0.35),
                Degrees.of(360)
            );

        // debug
        debugPoseTranslationsNear = () ->
            PoseUtil.isPoseTranslationNear(swerveSubsystem.getPose(), this.targetPoseSupplier.get(), positionTolerance);
        debugPoseRotationsNear = () ->
            PoseUtil.isPoseRotationNear(swerveSubsystem.getPose(), this.targetPoseSupplier.get(), angleTolerance);
        debugVelocityNear = () ->
            PoseUtil.isVelocityNear(swerveSubsystem.getVelocityMagnitude(), MetersPerSecond.of(0), speedTolerance);
    }

    /**
     * Sets a new target pose supplier.
     * @param newTargetPoseSupplier - The new target pose supplier.
     */
    public void setPoseSupplier(Supplier<Pose2d> newTargetPoseSupplier) {
        this.targetPoseSupplier = newTargetPoseSupplier;
    }

    // @Override
    public void initialize() {
        targetPose = targetPoseSupplier.get();

        // Reset controller state
        driveController.reset(swerveSubsystem.getPose(), new ChassisSpeeds());
    }

    // @Override
    // public void execute() {
    //     PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    //     goalState.pose = targetPoseSupplier.get();

    //     swerveSubsystem.runVelocityChassisSpeeds(
    //         driveController.calculateRobotRelativeSpeeds(swerveSubsystem.getPose(), goalState)
    //     );
    // }

    public ChassisSpeeds getChassisSpeeds() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = targetPoseSupplier.get();

        return driveController.calculateRobotRelativeSpeeds(swerveSubsystem.getPose(), goalState);
    }
    // @Override
    // public boolean isFinished() {
    //     return atTarget.getAsBoolean();
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     if (interrupted) {
    //         swerveSubsystem.stop();
    //     }
    // }
}
