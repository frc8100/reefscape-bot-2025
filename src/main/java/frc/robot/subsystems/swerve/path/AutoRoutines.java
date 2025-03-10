package frc.robot.subsystems.swerve.path;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.claw.ClawConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Contains the autonomous (and teleop) routines for the robot.
 */
public class AutoRoutines {

    /**
     * Locations of the robot to interact with the field elements.
     * Note: not flipped for the other side of the field.
     */
    public enum FieldLocations {
        REEF_1L(new Pose2d(3.74, 5, Rotation2d.fromDegrees(-60))),
        REEF_1R(new Pose2d(4, 5.15, Rotation2d.fromDegrees(-60))),

        REEF_2L(new Pose2d(4.95, 5.16, Rotation2d.fromDegrees(-120))),
        REEF_2R(new Pose2d(5.22, 5, Rotation2d.fromDegrees(-120))),

        REEF_3L(new Pose2d(5.7, 4.2, Rotation2d.fromDegrees(180))),
        REEF_3R(new Pose2d(5.7, 3.87, Rotation2d.fromDegrees(180))),

        REEF_4L(new Pose2d(5.24, 3.06, Rotation2d.fromDegrees(120))),
        REEF_4R(new Pose2d(4.96, 2.9, Rotation2d.fromDegrees(120))),

        REEF_5L(new Pose2d(4.03, 2.89, Rotation2d.fromDegrees(60))),
        REEF_5R(new Pose2d(3.75, 3.06, Rotation2d.fromDegrees(60))),

        REEF_6L(new Pose2d(3.28, 3.85, Rotation2d.fromDegrees(0))),
        REEF_6R(new Pose2d(3.28, 4.17, Rotation2d.fromDegrees(0))),

        CORAL_STATION_1(new Pose2d(1.1, 1.1, Rotation2d.fromDegrees(55))),
        CORAL_STATION_2(new Pose2d(1.1, 6.8, Rotation2d.fromDegrees(-55)));

        private Pose2d pose;

        /**
         * @return The pose of the location.
         */
        public Pose2d getPose() {
            return pose;
        }

        private FieldLocations(Pose2d pose) {
            this.pose = pose;
        }
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
     * @return A command to set up the superstructure for a given level by moving the elevator and claw to the correct positions.
     */
    public Command setUpSuperstructure(SuperstructureConstants.Level level) {
        return Commands.parallel(
            // TODO: Move claw out of the way first before moving elevator
            elevatorSubsystem.getPositionCommandAndWait(level.getElevatorDistance()),
            clawSubsystem.getWaitForAngleCommand(level.getClawAngle())
        );
    }

    /**
     * @return A command to pathfind to a given location.
     * @param location - The location to pathfind to. See {@link FieldLocations} for possible locations.
     */
    public Command pathFindToLocation(FieldLocations location) {
        // Get the command to go to the reef
        return AutoBuilder.pathfindToPose(location.getPose(), SwerveConfig.pathConstraints);
        // return new SequentialCommandGroup(
        //     AutoBuilder.pathfindToPose(location.getPose(), SwerveConfig.pathConstraints),
        //     Commands.waitSeconds(0.1)
        // );
    }

    /**
     * @return A command to get coral and go to all reefs, repeating the process.
     */
    public Command getCoralAndGoToAllReefsTest() {
        return new SequentialCommandGroup(
            // Get coral from station 1
            pathFindToLocation(FieldLocations.CORAL_STATION_1),
            // Run intake
            clawSubsystem.runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.INTAKE),
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
                setUpSuperstructure(SuperstructureConstants.Level.L1)
            ),
            // Run intake
            clawSubsystem.runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.INTAKE),
            // Simultaneously set up superstructure and pathfind to reef
            new ParallelCommandGroup(
                pathFindToLocation(FieldLocations.REEF_4R),
                Commands.waitSeconds(1.75).andThen(setUpSuperstructure(SuperstructureConstants.Level.L4))
            ),
            // Run outtake
            clawSubsystem.runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.OUTTAKE),
            // Reset superstructure
            setUpSuperstructure(SuperstructureConstants.Level.L1)
        );
    }
}
