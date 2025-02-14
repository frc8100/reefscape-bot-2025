package frc.robot.subsystems.swerve.path;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Contains the autonomous (and teleop) routines for the robot.
 */
public class AutoRoutines {
    private SwerveDrive swerveSubsystem;
    private SwervePathFollow pathFollow;

    public AutoRoutines(SwerveDrive swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.pathFollow = swerveSubsystem.getPathfindingInstance();
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
                AutoBuilder.pathfindToPose(SwervePathFollow.FieldLocations.coralStation1, SwerveConfig.pathConstraints),
                Commands.waitSeconds(3),
                AutoBuilder.pathfindToPose(SwervePathFollow.FieldLocations.reef1, SwerveConfig.pathConstraints));
    }
}
