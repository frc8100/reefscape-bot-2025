package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.lib.util.statemachine.StateMachine;
import frc.lib.util.statemachine.StateMachineState;
import frc.robot.Controls;
import frc.robot.commands.TeleopSwerve.DriveToPoseState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.SwerveState;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.path.AutoRoutines;
import java.util.Set;
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
     * Whether to dampen the input.
     */
    private BooleanSupplier dampen;

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
        BooleanSupplier dampen,
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
        this.dampen = dampen;
        this.speedDial = speedDial;
        this.logValues = logValues;

        driveToPoseCommand = new DriveToPose(this.swerveSubsystem, () -> new Pose2d());

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

        swerveSubsystem.stateMachine
            .getStateTrigger(SwerveState.FULL_DRIVER_CONTROL)
            .onTrue(
                Commands.runOnce(() -> {
                    if (pathFindToPoseCommand != null) {
                        pathFindToPoseCommand.cancel();
                        pathFindToPoseCommand = null;
                    }
                    stateMachine.scheduleStateChange(DriveToPoseState.NOT_DRIVING_TO_POSE);
                })
            );
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
            driveControls::isDampen,
            driveControls::getSpeedMultiplier,
            logValues
        );
    }

    public ChassisSpeeds getChassisSpeedsFromControls() {
        // Get values and deadband
        double translationInput = translationSupplier.getAsDouble();
        double strafeInput = strafeSupplier.getAsDouble();
        double rotationInput = rotationSupplier.getAsDouble();

        /**
         * The constant to multiply each value by
         */
        // TODO: refactor
        double dampenAndSpeedConstant = (dampen.getAsBoolean() ? 0.3 : 1) * (speedDial.getAsDouble());

        double translationValue =
            MathUtil.applyDeadband(translationInput, SwerveConfig.DRIVE_STICK_DEADBAND) * dampenAndSpeedConstant;
        double strafeValue =
            MathUtil.applyDeadband(strafeInput, SwerveConfig.DRIVE_STICK_DEADBAND) * dampenAndSpeedConstant;
        double rotationValue =
            MathUtil.applyDeadband(rotationInput, SwerveConfig.DRIVE_STICK_DEADBAND) * dampenAndSpeedConstant;

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
        ChassisSpeeds inputtedNudge = getChassisSpeedsFromControls();

        Translation2d nudgeTranslation = new Translation2d(
            inputtedNudge.vxMetersPerSecond,
            inputtedNudge.vyMetersPerSecond
        );

        Translation2d previousTranslation = new Translation2d(
            previous.vxMetersPerSecond,
            previous.vyMetersPerSecond
        );

        previousTranslation = previousTranslation.rotateBy(nudgeTranslation.getAngle().times(0.5).times(
            // Scale the nudge based on the magnitude of the nudge
            Math.hypot(nudgeTranslation.getX(), nudgeTranslation.getY()) / SwerveConfig.MAX_SPEED.in(MetersPerSecond)
        ));

        return new ChassisSpeeds(
            previousTranslation.getX(),
            previousTranslation.getY(),
            previous.omegaRadiansPerSecond + inputtedNudge.omegaRadiansPerSecond * 0.5
        );
    }

    /**
     * The function that is called to drive.
     */
    @Override
    public void execute() {
        
        switch (swerveSubsystem.stateMachine.getCurrentState().enumType) {
            case FULL_DRIVER_CONTROL:
                ChassisSpeeds desiredChassisSpeeds = getChassisSpeedsFromControls();
                swerveSubsystem.runVelocityChassisSpeeds(desiredChassisSpeeds);
                break;
            case DRIVE_TO_CORAL_STATION, DRIVE_TO_REEF:
                // TODO: change pose based on state
                // TODO: only set once
                Supplier<Pose2d> targetPoseSupplier = AutoRoutines.FieldLocations.CORAL_STATION_1::getPose;
                driveToPoseCommand.setPoseSupplier(targetPoseSupplier);

                switch (stateMachine.getCurrentState().enumType) {
                    case NOT_DRIVING_TO_POSE:
                        pathFindToPoseCommand = new PathfindingCommand(
                            targetPoseSupplier.get(),
                            SwerveConfig.pathConstraints,
                            swerveSubsystem::getPose,
                            swerveSubsystem::getChassisSpeeds,
                            (ChassisSpeeds speeds, DriveFeedforwards feedforwards) -> {
                                swerveSubsystem.runVelocityChassisSpeeds(applyInputNudge(speeds));
                            },
                            SwerveConfig.PP_INITIAL_PID_CONTROLLER,
                            SwerveConfig.getRobotConfig(),
                            swerveSubsystem
                        )
                            .raceWith(Commands.waitUntil(driveToPoseCommand.canSwitchToFinalAlignment))
                            .andThen(
                                Commands.runOnce(() ->
                                    stateMachine.scheduleStateChange(DriveToPoseState.FINAL_ALIGNMENT)
                                )
                            );

                        stateMachine.scheduleStateChange(DriveToPoseState.INITIAL_PATHFINDING);

                        pathFindToPoseCommand.schedule();
                        break;
                    case FINAL_ALIGNMENT:
                        swerveSubsystem.runVelocityChassisSpeeds(
                            applyInputNudge(driveToPoseCommand.getChassisSpeeds())
                        );
                        break;
                    case AT_TARGET:
                    case INITIAL_PATHFINDING:
                        // Do nothing, handled elsewhere
                        break;
                }
                break;
            default:
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
