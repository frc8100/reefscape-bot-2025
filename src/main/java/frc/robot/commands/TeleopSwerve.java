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
import frc.lib.util.statemachine.StateMachine;
import frc.lib.util.statemachine.StateMachineState;
import frc.robot.Controls;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.SwerveState;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.path.AutoRoutines;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The teleop swerve command.
 */
public class TeleopSwerve extends Command {

    public enum DriveToPoseState {
        /**
         * The drive to pose command is idle.
         * Usually means the robot is in full driver control.
         */
        NOT_DRIVING_TO_POSE,

        /**
         * The robot is performing initial pathfinding to the target pose.
         */
        INITIAL_PATHFINDING,

        /**
         * The robot is performing final alignment to the target pose using a simple PID controller.
         */
        FINAL_ALIGNMENT,

        /**
         * The robot has reached the target pose.
         */
        AT_TARGET,
    }

    public final StateMachine<DriveToPoseState> stateMachine;

    /**
     * The drive to pose command used for final alignment.
     */
    public final DriveToPose driveToPoseCommand;

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
        addRequirements(swerveSubsystem);

        // Set the values from the constructor
        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.robotCentricSupplier = robotCentricSupplier;
        this.speedDial = speedDial;
        this.logValues = logValues;

        driveToPoseCommand = new DriveToPose(this.swerveSubsystem, () -> new Pose2d());

        // Set up the state machine
        stateMachine = new StateMachine<DriveToPoseState>("Swerve/DriveToPose")
            .withDefaultState(new StateMachineState<>(DriveToPoseState.NOT_DRIVING_TO_POSE, "Idle"))
            .withState(
                new StateMachineState<>(
                    DriveToPoseState.INITIAL_PATHFINDING,
                    "Initial Pathfinding",
                    (DriveToPoseState previousState) -> previousState == DriveToPoseState.NOT_DRIVING_TO_POSE
                )
            )
            .withState(
                new StateMachineState<>(
                    DriveToPoseState.FINAL_ALIGNMENT,
                    "Final Alignment",
                    (DriveToPoseState previousState) ->
                        previousState == DriveToPoseState.INITIAL_PATHFINDING &&
                        driveToPoseCommand.canSwitchToFinalAlignment.getAsBoolean()
                )
            );

        // When we enter full driver control, cancel any pathfinding commands and set state to not driving to pose
        swerveSubsystem.stateMachine.addOnStateChangeAction(SwerveState.FULL_DRIVER_CONTROL, () -> {
            if (pathFindToPoseCommand != null) {
                pathFindToPoseCommand.cancel();
                pathFindToPoseCommand = null;
            }

            stateMachine.scheduleStateChange(DriveToPoseState.NOT_DRIVING_TO_POSE);
        });
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
     * @return The chassis speeds based on controller input, scaled to {@link SwerveConfig#MAX_SPEED} and {@link SwerveConfig#MAX_ANGULAR_VELOCITY}.
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
            rotationValue * SwerveConfig.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
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

    /**
     * The function that is called to drive.
     */
    @Override
    public void execute() {
        switch (swerveSubsystem.stateMachine.getCurrentState().enumType) {
            case FULL_DRIVER_CONTROL:
                // Drive based on the raw controller inputs
                ChassisSpeeds desiredChassisSpeeds = getChassisSpeedsFromControls();
                swerveSubsystem.runVelocityChassisSpeeds(desiredChassisSpeeds);
                break;
            // Semi-autonomous states
            case DRIVE_TO_CORAL_STATION, DRIVE_TO_REEF:
                // TODO: change pose based on state
                // TODO: only set once
                driveSemiAuto(AutoRoutines.FieldLocations.CORAL_STATION_1::getPose);
                break;
            default:
                break;
        }
    }

    /**
     * Drives semi-autonomously to a target pose based on the state machine.
     * @param targetPoseSupplier - The supplier for the target pose.
     */
    private void driveSemiAuto(Supplier<Pose2d> targetPoseSupplier) {
        driveToPoseCommand.setPoseSupplier(targetPoseSupplier);

        switch (stateMachine.getCurrentState().enumType) {
            case NOT_DRIVING_TO_POSE:
                // Initialize pathfinding to the target pose (if not already doing so)
                pathFindToPoseCommand = new PathfindingCommand(
                    targetPoseSupplier.get(),
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
                        Commands.runOnce(() -> stateMachine.scheduleStateChange(DriveToPoseState.FINAL_ALIGNMENT))
                    );

                // Schedule state change to initial pathfinding so the command isn't scheduled multiple times
                stateMachine.scheduleStateChange(DriveToPoseState.INITIAL_PATHFINDING);

                pathFindToPoseCommand.schedule();
                break;
            case FINAL_ALIGNMENT:
                // Run final alignment
                swerveSubsystem.runVelocityChassisSpeeds(applyInputNudge(driveToPoseCommand.getChassisSpeeds()));

                // If at target, switch state
                if (driveToPoseCommand.atTarget.getAsBoolean()) {
                    stateMachine.scheduleStateChange(DriveToPoseState.AT_TARGET);
                }
                break;
            case AT_TARGET:
                // If the robot is ever not at the target, go back to final alignment
                if (!driveToPoseCommand.atTarget.getAsBoolean()) {
                    stateMachine.scheduleStateChange(DriveToPoseState.FINAL_ALIGNMENT);
                }
                break;
            case INITIAL_PATHFINDING:
                // Do nothing, handled elsewhere
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Drive empty values
        if (interrupted) {
            swerveSubsystem.drive(new Translation2d(), 0, false);
        }
    }
}
