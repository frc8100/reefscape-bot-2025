package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.util.PoseUtil;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Uses a PID controller to drive the robot to a specified pose. Does not use path finding/obstacle avoidance.
 */
public class DriveToPosePID {

    /**
     * A record containing whether the robot is at each target.
     */
    public record IsAtTargets(
        boolean atTarget,
        boolean atPoseTranslationTarget,
        boolean atPoseRotationTarget,
        boolean atVelocityTarget
    ) {
        public static final IsAtTargets ALL_FALSE = new IsAtTargets(false, false, false, false);
    }

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
    private static final LinearVelocity targetVelocity = MetersPerSecond.of(0);

    // Final alignment only checks position so the tolerance is any (360 degrees)
    private static final Angle finalAlignmentAngleTolerance = Degrees.of(360);

    private final SwerveDrive swerveSubsystem;

    /**
     * A supplier that provides the target pose to drive to.
     * Called once per command execution.
     */
    private Supplier<Pose2d> targetPoseSupplier;

    /**
     * A trigger that is true when the robot is at the target pose.
     */
    public final BooleanSupplier atTarget;

    public final BooleanSupplier atPoseTranslationTarget;
    public final BooleanSupplier atPoseRotationTarget;
    public final BooleanSupplier atVelocityTarget;

    /**
     * Whether the robot can switch to final alignment mode.
     */
    public final BooleanSupplier canSwitchToFinalAlignment;

    /**
     * The goal state for the drive controller.
     */
    private final PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();

    /**
     * Creates a new DriveToPose command.
     * @param swerveSubsystem - The swerve drive subsystem.
     * @param targetPoseSupplier - A supplier that provides the target pose to drive to. See {@link #targetPoseSupplier}.
     */
    public DriveToPosePID(Swerve swerveSubsystem, Supplier<Pose2d> targetPoseSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;

        atTarget = () ->
            PoseUtil.isPosesAndVelocityNear(
                swerveSubsystem.getPose(),
                this.targetPoseSupplier.get(),
                swerveSubsystem.getVelocityMagnitude(),
                targetVelocity,
                positionTolerance,
                angleTolerance,
                speedTolerance
            );

        canSwitchToFinalAlignment = () ->
            PoseUtil.isNear(
                swerveSubsystem.getPose(),
                this.targetPoseSupplier.get(),
                // Start final alignment when within this distance plus a bit based on current speed
                0.4 + swerveSubsystem.getVelocityMagnitude().in(MetersPerSecond) * 0.35,
                finalAlignmentAngleTolerance.in(Radians)
            );

        // debug
        atPoseTranslationTarget = () ->
            PoseUtil.isPoseTranslationNear(swerveSubsystem.getPose(), this.targetPoseSupplier.get(), positionTolerance);
        atPoseRotationTarget = () ->
            PoseUtil.isPoseRotationNear(swerveSubsystem.getPose(), this.targetPoseSupplier.get(), angleTolerance);
        atVelocityTarget = () ->
            PoseUtil.isVelocityNear(swerveSubsystem.getVelocityMagnitude(), targetVelocity, speedTolerance);
    }

    public IsAtTargets getAtTargetsRecords() {
        return new IsAtTargets(
            atTarget.getAsBoolean(),
            atPoseTranslationTarget.getAsBoolean(),
            atPoseRotationTarget.getAsBoolean(),
            atVelocityTarget.getAsBoolean()
        );
    }

    /**
     * Sets a new target pose supplier.
     * @param newTargetPoseSupplier - The new target pose supplier.
     */
    public void setPoseSupplier(Supplier<Pose2d> newTargetPoseSupplier) {
        this.targetPoseSupplier = newTargetPoseSupplier;
    }

    /**
     * Sets a new target pose supplier.
     * @param newTargetPoseSupplier - The new target pose supplier.
     */
    public void setOptionalPoseSupplier(Optional<Supplier<Pose2d>> newTargetPoseSupplier) {
        this.targetPoseSupplier = newTargetPoseSupplier.isPresent()
            ? newTargetPoseSupplier.get()
            : swerveSubsystem::getPose;
    }

    /**
     * Calculates the chassis speeds needed to drive to the target pose.
     * @return The chassis speeds needed to drive to the target pose.
     */
    public ChassisSpeeds getChassisSpeeds() {
        goalState.pose = targetPoseSupplier.get();

        return driveController.calculateRobotRelativeSpeeds(swerveSubsystem.getPose(), goalState);
    }
}
