package frc.robot.subsystems.swerve.path;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Contains the autonomous (and teleop) routines for the robot.
 */
public class AutoRoutines {
    /**
     * Locations of the robot to interact with the field elements.
     * Note: not flipped for the blue side.
     */
    public static class FieldLocations {
        public static final Pose2d coralStation1 = new Pose2d(1.1, 1.1, Rotation2d.fromDegrees(55));
        public static final Pose2d coralStation2 = new Pose2d(1.1, 6.8, Rotation2d.fromDegrees(-55));

        public static final Pose2d reef1 = new Pose2d(3.6, 5.5, Rotation2d.fromDegrees(-60));
    }

    private SwerveDrive swerveSubsystem;

    public AutoRoutines(SwerveDrive swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    /**
     * @return A command to drive to the coral station 1 and wait for coral from the station.
     */
    public Command getCoralFromStation1() {
        // return new SequentialCommandGroup(
        //         pathFollow.generateGoToPoseCommand(SwervePathFollow.FieldLocations.coralStation1),
        //         Commands.waitSeconds(3),
        //         // TODO: test
        //         pathFollow.generateGoToPoseCommand(SwervePathFollow.FieldLocations.reef1));

        return new SequentialCommandGroup(
                AutoBuilder.pathfindToPose(FieldLocations.coralStation1, SwerveConfig.pathConstraints),
                Commands.waitSeconds(3),
                AutoBuilder.pathfindToPose(FieldLocations.reef1, SwerveConfig.pathConstraints));
    }
}
