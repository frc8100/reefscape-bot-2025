package frc.robot.subsystems.swerve.path;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.PoseUtil;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.claw.ClawConstants;
import frc.robot.subsystems.superstructure.claw.ClawConstants.IntakeOuttakeDirection;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Contains the autonomous (and teleop) routines for the robot.
 */
public class AutoRoutines {

    /**
     * The types of field locations.
     */
    public enum FieldLocationType {
        LEFT_REEF,
        RIGHT_REEF,
        CORAL_STATION,
        PROCESSOR,
    }

    /**
     * Locations of the robot to interact with the field elements.
     * Pose is relative to the blue alliance.
     */
    public enum FieldLocations {
        REEF_1L(new Pose2d(3.74, 5, Rotation2d.fromDegrees(-60)), FieldLocationType.LEFT_REEF),
        REEF_1R(new Pose2d(4, 5.15, Rotation2d.fromDegrees(-60)), FieldLocationType.RIGHT_REEF),

        REEF_2L(new Pose2d(4.95, 5.16, Rotation2d.fromDegrees(-120)), FieldLocationType.LEFT_REEF),
        REEF_2R(new Pose2d(5.22, 5, Rotation2d.fromDegrees(-120)), FieldLocationType.RIGHT_REEF),

        REEF_3L(new Pose2d(5.7, 4.2, Rotation2d.fromDegrees(180)), FieldLocationType.LEFT_REEF),
        REEF_3R(new Pose2d(5.7, 3.87, Rotation2d.fromDegrees(180)), FieldLocationType.RIGHT_REEF),

        REEF_4L(new Pose2d(5.24, 3.06, Rotation2d.fromDegrees(120)), FieldLocationType.LEFT_REEF),
        REEF_4R(new Pose2d(4.96, 2.9, Rotation2d.fromDegrees(120)), FieldLocationType.RIGHT_REEF),

        REEF_5L(new Pose2d(4.03, 2.89, Rotation2d.fromDegrees(60)), FieldLocationType.LEFT_REEF),
        REEF_5R(new Pose2d(3.75, 3.06, Rotation2d.fromDegrees(60)), FieldLocationType.RIGHT_REEF),

        REEF_6L(new Pose2d(3.28, 3.85, Rotation2d.fromDegrees(0)), FieldLocationType.LEFT_REEF),
        REEF_6R(new Pose2d(3.28, 4.17, Rotation2d.fromDegrees(0)), FieldLocationType.RIGHT_REEF),

        CORAL_STATION_1(new Pose2d(1.1, 1.1, Rotation2d.fromDegrees(55)), FieldLocationType.CORAL_STATION),
        CORAL_STATION_2(new Pose2d(1.1, 6.8, Rotation2d.fromDegrees(-55)), FieldLocationType.CORAL_STATION),

        PROCESSOR(new Pose2d(6.732, 0.652, Rotation2d.fromDegrees(-90)), FieldLocationType.PROCESSOR);

        // A list of all the left reef poses and right reef poses, for easy access
        private static final List<Pose2d> leftReefPoses;
        private static final List<Pose2d> rightReefPoses;
        private static final List<Pose2d> coralStationPoses;

        // Initialize the lists of reef poses
        static {
            leftReefPoses = List.of(
                REEF_1L.getPose(),
                REEF_2L.getPose(),
                REEF_3L.getPose(),
                REEF_4L.getPose(),
                REEF_5L.getPose(),
                REEF_6L.getPose()
            );

            rightReefPoses = List.of(
                REEF_1R.getPose(),
                REEF_2R.getPose(),
                REEF_3R.getPose(),
                REEF_4R.getPose(),
                REEF_5R.getPose(),
                REEF_6R.getPose()
            );

            coralStationPoses = List.of(CORAL_STATION_1.getPose(), CORAL_STATION_2.getPose());
        }

        public static List<Pose2d> getLeftReefPoses() {
            return leftReefPoses;
        }

        public static List<Pose2d> getRightReefPoses() {
            return rightReefPoses;
        }

        public static List<Pose2d> getCoralStationPoses() {
            return coralStationPoses;
        }

        private Pose2d pose;
        private FieldLocationType type;

        /**
         * @return The pose of the location. Automatically flips if necessary.
         * @param shouldFlip - Whether or not to flip the pose. Defaults to true.
         * Also takes into account the current alliance color.
         */
        public Pose2d getPose(boolean shouldFlip) {
            return (shouldFlip && PoseUtil.shouldFlip()) ? FlippingUtil.flipFieldPose(pose) : pose;
        }

        public Pose2d getPose() {
            return getPose(true);
        }

        /**
         * @return The type of the location.
         */
        public FieldLocationType getType() {
            return type;
        }

        private FieldLocations(Pose2d pose, FieldLocationType type) {
            this.pose = pose;
            this.type = type;
        }
    }

    /**
     * @return The pose of the nearest reef to the robot's current location.
     */
    private static Pose2d getNearestLeftReefGivenPosition(Pose2d currentPose) {
        return currentPose.nearest(FieldLocations.getLeftReefPoses());
    }

    private static Pose2d getNearestRightReefGivenPosition(Pose2d currentPose) {
        return currentPose.nearest(FieldLocations.getRightReefPoses());
    }

    private static Pose2d getNearestCoralStationGivenPosition(Pose2d currentPose) {
        return currentPose.nearest(FieldLocations.getCoralStationPoses());
    }

    // References to subsystems
    private SwerveDrive swerveSubsystem;
    private Elevator elevatorSubsystem;
    private Claw clawSubsystem;

    /**
     * Creates a new AutoRoutines object given required subsystems.
     */
    public AutoRoutines(SwerveDrive swerveSubsystem, Elevator elevatorSubsystem, Claw clawSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
    }

    /**
     * @return The pose of the nearest left reef to the robot's current location.
     */
    public Pose2d getNearestLeftReef() {
        return getNearestLeftReefGivenPosition(swerveSubsystem.getPose());
    }

    /**
     * @return The pose of the nearest right reef to the robot's current location.
     */
    public Pose2d getNearestRightReef() {
        return getNearestRightReefGivenPosition(swerveSubsystem.getPose());
    }

    public Pose2d getNearestCoralStation() {
        return getNearestCoralStationGivenPosition(swerveSubsystem.getPose());
    }

    /**
     * @return A command to set up the superstructure for a given level by moving the elevator and claw to the correct positions.
     */
    public Command setUpSuperstructure(SuperstructureConstants.Level level) {
        return clawSubsystem
            // Move claw out of way
            .getWaitForAngleCommand(ClawConstants.RotationPositions.CLAW_HOLDING_POSITION)
            // Raise elevator
            .andThen(elevatorSubsystem.getPositionCommandAndWait(level))
            // Move claw to level angle
            .andThen(clawSubsystem.getWaitForAngleCommand(level.getClawAngle()))
            // Stop when interrupted
            .handleInterrupt(() -> {
                elevatorSubsystem.io.resetSetpointToCurrentPosition();
                clawSubsystem.io.resetSetpointToCurrentPosition();
            });
    }

    /**
     * @return A command to do the elevator and claw movements for the intake of algae from L2.
     */
    public Command intakeAlgae(SuperstructureConstants.Level algaeLevel) {
        // Move superstructure up
        return setUpSuperstructure(algaeLevel)
            .andThen(
                new ParallelCommandGroup(
                    // Intake and hold algae
                    clawSubsystem.runIntakeOrOuttake(IntakeOuttakeDirection.BACK, ClawConstants.ALGAE_TIMEOUT_TIME),
                    // Raise elevator a bit
                    Commands.run(() -> elevatorSubsystem.runMotor(1))
                        .withTimeout(Seconds.of(0.5))
                        // Lower elevator
                        .andThen(
                            elevatorSubsystem.getPositionCommandAndWait(SuperstructureConstants.Level.INITIAL_POSITION)
                        )
                )
            )
            // Stop when interrupted
            .handleInterrupt(() -> {
                elevatorSubsystem.io.resetSetpointToCurrentPosition();
                clawSubsystem.io.resetSetpointToCurrentPosition();
            });
    }

    /**
     * @return A command to do only the claw movements for L4.
     */
    public Command scoreL4() {
        return clawSubsystem
            .getWaitForAngleCommand(ClawConstants.RotationPositions.CLAW_L4_SCORING_POSITION)
            .andThen(clawSubsystem.runOuttakeUntilCoralIsNotInClaw())
            .andThen(clawSubsystem.getWaitForAngleCommand(SuperstructureConstants.Level.L4.getClawAngle()))
            // Stop when interrupted
            .handleInterrupt(() -> {
                clawSubsystem.io.resetSetpointToCurrentPosition();
                clawSubsystem.io.runOuttake(0);
            });
    }

    /**
     * @return A command to pathfind to a given location.
     * @param location - The location to pathfind to. See {@link FieldLocations} for possible locations.
     * @param shouldAlignToReef - Whether or not to align to the reef tag after pathfinding.
     */
    public Command pathFindToLocation(FieldLocations location, boolean shouldAlignToReef) {
        // Get the command to go to the reef
        Command initialPathfindCommand = AutoBuilder.pathfindToPose(
            // Do not flip the pose, as it is already flipped
            location.getPose(false),
            SwerveConfig.pathConstraints
        );

        // If the location is a reef, do the final alignment
        if (
            shouldAlignToReef &&
            (location.getType() == FieldLocationType.LEFT_REEF || location.getType() == FieldLocationType.RIGHT_REEF)
        ) {
            return initialPathfindCommand.andThen(
                new AlignToReefTagRelative(location.getType() == FieldLocationType.RIGHT_REEF, swerveSubsystem)
            );
        }

        return initialPathfindCommand;
    }

    public Command pathFindToLocation(FieldLocations location) {
        return pathFindToLocation(location, true);
    }

    public Command pathFindToLocation(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, SwerveConfig.pathConstraints);
    }

    public Command continuouslyPathFindToLocation(Supplier<Pose2d> pose) {
        return new DeferredCommand(
            () ->
                pathFindToLocation(pose.get())
                    .andThen(Commands.waitSeconds(0.1))
                    .andThen(continuouslyPathFindToLocation(pose)),
            Set.of(swerveSubsystem)
        );
    }

    /**
     * @return A command to move forward for a given amount of time at a given speed.
     * @param vxMetersPerSecond - The speed to move forward at in meters per second.
     * @param timeSeconds - The amount of time to move forward for in seconds.
     */
    private Command driveForwardWithSpeedFor(double vxMetersPerSecond, double timeSeconds) {
        return Commands.deadline(
            Commands.waitSeconds(timeSeconds),
            Commands.run(
                () -> swerveSubsystem.runVelocityChassisSpeeds(new ChassisSpeeds(vxMetersPerSecond, 0, 0)),
                swerveSubsystem
            )
        );
    }

    /**
     * @return A command that can be used in auto to move forward for a set amount of time.
     * More reliable than pathfinding.
     */
    public Command actuallyMoveForward() {
        return driveForwardWithSpeedFor(1, 2.25);
    }

    public Command moveForwardAndL1() {
        return driveForwardWithSpeedFor(0.75, 2.25)
            .alongWith(setUpSuperstructure(SuperstructureConstants.Level.L1_AUTO))
            .andThen(clawSubsystem.runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.BACK));
    }

    /**
     * @return Align with the reef, and score on the level.
     */
    public Command alignAndScore(SuperstructureConstants.Level levelToScoreOn) {
        return new AlignToReefTagRelative(false, swerveSubsystem)
            .alongWith(setUpSuperstructure(levelToScoreOn))
            .andThen(clawSubsystem.runOuttakeUntilCoralIsNotInClaw());
    }

    /**
     * @return A command to get coral and go to all reefs, repeating the process.
     */
    public Command getCoralAndGoToAllReefsTest() {
        return new SequentialCommandGroup(
            // Get coral from station 1
            pathFindToLocation(FieldLocations.CORAL_STATION_1),
            // Run intake
            clawSubsystem.runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.BACK),
            // Simultaneously set up superstructure and pathfind to reef
            new ParallelCommandGroup(
                pathFindToLocation(FieldLocations.REEF_4L),
                Commands.waitSeconds(1.75).andThen(setUpSuperstructure(SuperstructureConstants.Level.L4))
            ),
            // Run outtake
            clawSubsystem.runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.OUTTAKE),
            // Reset superstructure
            // Simultaneously set up superstructure and pathfind to coral station
            new ParallelCommandGroup(
                pathFindToLocation(FieldLocations.CORAL_STATION_1),
                setUpSuperstructure(SuperstructureConstants.Level.INITIAL_POSITION)
            ),
            // Run intake
            clawSubsystem.runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.BACK),
            // Simultaneously set up superstructure and pathfind to reef
            new ParallelCommandGroup(
                pathFindToLocation(FieldLocations.REEF_4R),
                Commands.waitSeconds(1.75).andThen(setUpSuperstructure(SuperstructureConstants.Level.L4))
            ),
            // Run outtake
            clawSubsystem.runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.OUTTAKE),
            // Reset superstructure
            setUpSuperstructure(SuperstructureConstants.Level.INITIAL_POSITION)
        );
    }
}
