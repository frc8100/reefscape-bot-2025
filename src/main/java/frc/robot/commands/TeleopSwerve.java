package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.SwerveState;
import frc.robot.subsystems.swerve.SwerveConfig;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The teleop swerve command.
 */
public class TeleopSwerve {

    /**
     * The drive to pose command used for final alignment.
     */
    public final DriveToPosePID driveToPoseCommand;

    private Command pathFindToPoseCommand = null;

    /**
     * The swerve subsystem.
     */
    private final Swerve swerveSubsystem;

    /**
     * The translation input (x), as a double from 0-1.
     * Should be a controller/joystick input.
     */
    private DoubleSupplier translationSupplier;

    /**
     * The strafe input (y), as a double from 0-1.
     * Should be a controller/joystick input.
     */
    private DoubleSupplier strafeSupplier;

    /**
     * The rotation input, as a double from 0-1.
     * Should be a controller/joystick input.
     */
    private DoubleSupplier rotationSupplier;

    /**
     * Whether the swerve is robot centric.
     * Default is always `false`.
     */
    private BooleanSupplier robotCentricSupplier;

    /**
     * The speed multiplier, as a double.
     * Default is `1`
     */
    private DoubleSupplier speedDial;

    /**
     * Whether to log values of the drive. Default is `true`
     */
    private boolean logValues;

    /**
     * Creates the TeleopSwerve command.
     * The parameters are members of this class.
     */
    public TeleopSwerve(
        Swerve swerveSubsystem,
        DoubleSupplier translationSupplier,
        DoubleSupplier strafeSupplier,
        DoubleSupplier rotationSupplier,
        BooleanSupplier robotCentricSupplier,
        DoubleSupplier speedDial,
        boolean logValues
    ) {
        this.swerveSubsystem = swerveSubsystem;

        // Set the values from the constructor
        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.robotCentricSupplier = robotCentricSupplier;
        this.speedDial = speedDial;
        this.logValues = logValues;

        driveToPoseCommand = new DriveToPosePID(this.swerveSubsystem, this.swerveSubsystem::getPose);

        logCurrentStates();

        // When we enter full driver control, cancel any pathfinding commands and set state to not driving to pose
        swerveSubsystem.stateMachine.addOnStateChangeAction(SwerveState.FULL_DRIVER_CONTROL, () -> {
            if (pathFindToPoseCommand != null) {
                pathFindToPoseCommand.cancel();
                pathFindToPoseCommand = null;
            }
        });

        // Bindings
        swerveSubsystem.stateMachine.addStateAction(SwerveState.FULL_DRIVER_CONTROL, this::driveFullDriverControl);
        swerveSubsystem.stateMachine.addStateAction(
            SwerveState.DRIVE_TO_POSE_PATHFINDING,
            this::getInitialPathfindingCommand
        );
        swerveSubsystem.stateMachine.addStateAction(SwerveState.DRIVE_TO_POSE_PID, this::getFinalAlignmentCommand);
        swerveSubsystem.stateMachine.addStateAction(SwerveState.DRIVE_TO_POSE_AT_TARGET, this::getAtTargetCommand);
    }

    /**
     * Creates the TeleopSwerve command given a {@link Controls.Drive} object.
     */
    public TeleopSwerve(Swerve swerveSubsystem, Controls.Drive driveControls, boolean logValues) {
        this(
            swerveSubsystem,
            driveControls::getTranslationAxis,
            driveControls::getStrafeAxis,
            driveControls::getRotationAxis,
            driveControls::isRobotCentric,
            driveControls::getSpeedMultiplier,
            logValues
        );
    }

    /**
     * @return The chassis speeds based on controller input, scaled to {@link SwerveConfig#MAX_SPEED} and {@link SwerveConfig#ANGULAR_VELOCITY_FOR_TELEOP}.
     */
    public ChassisSpeeds getChassisSpeedsFromControls() {
        // Get values from suppliers
        double translationInput = translationSupplier.getAsDouble();
        double strafeInput = strafeSupplier.getAsDouble();
        double rotationInput = rotationSupplier.getAsDouble();

        /**
         * The constant to multiply each value by
         */
        double speedMultiplier = speedDial.getAsDouble();

        double translationValue =
            MathUtil.applyDeadband(translationInput, SwerveConfig.DRIVE_STICK_DEADBAND) * speedMultiplier;
        double strafeValue = MathUtil.applyDeadband(strafeInput, SwerveConfig.DRIVE_STICK_DEADBAND) * speedMultiplier;
        double rotationValue =
            MathUtil.applyDeadband(rotationInput, SwerveConfig.DRIVE_STICK_DEADBAND) * speedMultiplier;

        // Log the values
        if (logValues) {
            Logger.recordOutput("Swerve/TranslationInput", translationInput);
            Logger.recordOutput("Swerve/StrafeInput", strafeInput);
            Logger.recordOutput("Swerve/RotationInput", rotationInput);

            Logger.recordOutput("Swerve/TranslationValue", translationValue);
            Logger.recordOutput("Swerve/StrafeValue", strafeValue);
            Logger.recordOutput("Swerve/RotationValue", rotationValue);
        }

        return swerveSubsystem.getSpeedsFromTranslation(
            new Translation2d(translationValue, strafeValue).times(SwerveConfig.MAX_SPEED.in(MetersPerSecond)),
            rotationValue * SwerveConfig.ANGULAR_VELOCITY_FOR_TELEOP.in(RadiansPerSecond),
            !robotCentricSupplier.getAsBoolean()
        );
    }

    /**
     * @param previous - The previous chassis speeds.
     * @return Apply a nudge to the previous chassis speeds based on controller input.
     */
    private ChassisSpeeds applyInputNudge(ChassisSpeeds previous) {
        // Get the nudge from the controller (as field centric)
        ChassisSpeeds inputtedNudge = getChassisSpeedsFromControls();

        // Convert to Translation2d to rotate around
        Translation2d previousTranslation = new Translation2d(previous.vxMetersPerSecond, previous.vyMetersPerSecond);

        Rotation2d rotateBy =
            // Use atan2 directly to avoid "x and y components of Rotation2d are zero" error
            new Rotation2d(Math.atan2(inputtedNudge.vyMetersPerSecond, inputtedNudge.vxMetersPerSecond))
                // Get the difference between the two angles
                .minus(new Rotation2d(Math.atan2(previous.vyMetersPerSecond, previous.vxMetersPerSecond)))
                .times(
                    // Scale down by the nudge input multiplier (avoid overshooting)
                    SwerveConfig.NUDGE_TRANSLATION_INPUT_MULTIPLIER *
                    // Scale the nudge based on the magnitude of the nudge
                    (Math.hypot(inputtedNudge.vxMetersPerSecond, inputtedNudge.vyMetersPerSecond) /
                        SwerveConfig.MAX_SPEED.in(MetersPerSecond))
                );

        Logger.recordOutput("Swerve/NudgeRotateBy", rotateBy.getDegrees());

        previousTranslation = previousTranslation.rotateBy(rotateBy);

        return new ChassisSpeeds(
            previousTranslation.getX(),
            previousTranslation.getY(),
            // Also nudge omega
            previous.omegaRadiansPerSecond +
            (inputtedNudge.omegaRadiansPerSecond * SwerveConfig.NUDGE_ROTATION_INPUT_MULTIPLIER)
        );
    }

    private void driveFullDriverControl() {
        // Drive based on the raw controller inputs
        ChassisSpeeds desiredChassisSpeeds = getChassisSpeedsFromControls();
        swerveSubsystem.runVelocityChassisSpeeds(desiredChassisSpeeds);
    }

    /**
     * Logs the current states of the drive to pose command.
     */
    private void logCurrentStates() {
        Logger.recordOutput(
            swerveSubsystem.stateMachine.dashboardKey + "/AtTarget",
            driveToPoseCommand.atTarget.getAsBoolean()
        );
        Logger.recordOutput(
            swerveSubsystem.stateMachine.dashboardKey + "/IsTranslationNear",
            driveToPoseCommand.debugPoseTranslationsNear.getAsBoolean()
        );
        Logger.recordOutput(
            swerveSubsystem.stateMachine.dashboardKey + "/IsRotationNear",
            driveToPoseCommand.debugPoseRotationsNear.getAsBoolean()
        );
        Logger.recordOutput(
            swerveSubsystem.stateMachine.dashboardKey + "/IsVelocityNear",
            driveToPoseCommand.debugVelocityNear.getAsBoolean()
        );
    }

    private void getInitialPathfindingCommand(Optional<Supplier<Pose2d>> targetPoseSupplier) {
        if (targetPoseSupplier.isEmpty()) {
            // No target pose, return to full driver control
            return;
        }

        Pose2d targetPose = targetPoseSupplier.get().get();

        Logger.recordOutput(swerveSubsystem.stateMachine.dashboardKey + "/TargetPose", targetPose);

        // Initialize pathfinding to the target pose (if not already doing so)
        pathFindToPoseCommand = new PathfindingCommand(
            targetPose,
            SwerveConfig.pathConstraints,
            swerveSubsystem::getPose,
            swerveSubsystem::getChassisSpeeds,
            (ChassisSpeeds speeds, DriveFeedforwards feedforwards) ->
                swerveSubsystem.runVelocityChassisSpeeds(applyInputNudge(speeds)),
            SwerveConfig.PP_INITIAL_PID_CONTROLLER,
            SwerveConfig.getRobotConfig(),
            swerveSubsystem
        )
            // End when we can switch to final alignment
            .raceWith(Commands.waitUntil(driveToPoseCommand.canSwitchToFinalAlignment))
            .andThen(
                Commands.runOnce(() -> swerveSubsystem.stateMachine.scheduleStateChange(SwerveState.DRIVE_TO_POSE_PID))
            );

        pathFindToPoseCommand.schedule();
    }

    private void getFinalAlignmentCommand(Optional<Supplier<Pose2d>> targetPoseSupplier) {
        driveToPoseCommand.setOptionalPoseSupplier(targetPoseSupplier);

        // Run final alignment
        swerveSubsystem.runVelocityChassisSpeeds(applyInputNudge(driveToPoseCommand.getChassisSpeeds()));

        // If at target, switch state
        if (driveToPoseCommand.atTarget.getAsBoolean()) {
            swerveSubsystem.stateMachine.scheduleStateChange(SwerveState.DRIVE_TO_POSE_AT_TARGET);
        }

        logCurrentStates();
    }

    private void getAtTargetCommand() {
        // If the robot is ever not at the target, go back to final alignment
        if (!driveToPoseCommand.atTarget.getAsBoolean()) {
            swerveSubsystem.stateMachine.scheduleStateChange(SwerveState.DRIVE_TO_POSE_PID);
        }

        swerveSubsystem.stop();
    }
}
