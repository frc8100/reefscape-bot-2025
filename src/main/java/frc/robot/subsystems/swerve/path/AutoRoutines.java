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

        public static final Pose2d reef1 = new Pose2d(3.7, 5.2, Rotation2d.fromDegrees(-60));
        public static final Pose2d reef2 = new Pose2d(5.3, 5.2, Rotation2d.fromDegrees(-120));
        public static final Pose2d reef3 = new Pose2d(6, 4, Rotation2d.fromDegrees(180));
        public static final Pose2d reef4 = new Pose2d(5.25, 2.8, Rotation2d.fromDegrees(120));
        public static final Pose2d reef5 = new Pose2d(3.7, 2.7, Rotation2d.fromDegrees(60));
        public static final Pose2d reef6 = new Pose2d(3, 4, Rotation2d.fromDegrees(0));
    }

    private SwerveDrive swerveSubsystem;

    public AutoRoutines(SwerveDrive swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    /**
     * @return A command to drive to the coral station 1 and wait for coral from the station.
     */
    public Command getCoralFromStation(int station) {
        // Get the command to go to the coral station
        Command goToCoralStationCommand;
        switch (station) {
            case 1:
                goToCoralStationCommand =
                        AutoBuilder.pathfindToPose(FieldLocations.coralStation1, SwerveConfig.pathConstraints);
                break;
            case 2:
                goToCoralStationCommand =
                        AutoBuilder.pathfindToPose(FieldLocations.coralStation2, SwerveConfig.pathConstraints);
                break;
            default:
                goToCoralStationCommand = Commands.run(() -> {
                    System.out.println("Invalid station number: " + station);
                });
        }

        return new SequentialCommandGroup(goToCoralStationCommand, Commands.waitSeconds(0.5));
    }

    /**
     * @return A command to drive to a reef and wait for the robot to be ready to score.
     */
    public Command goToReef(int reef) {
        // Get the command to go to the reef
        Command goToReefCommand;
        switch (reef) {
            case 1:
                goToReefCommand = AutoBuilder.pathfindToPose(FieldLocations.reef1, SwerveConfig.pathConstraints);
                break;
            case 2:
                goToReefCommand = AutoBuilder.pathfindToPose(FieldLocations.reef2, SwerveConfig.pathConstraints);
                break;
            case 3:
                goToReefCommand = AutoBuilder.pathfindToPose(FieldLocations.reef3, SwerveConfig.pathConstraints);
                break;
            case 4:
                goToReefCommand = AutoBuilder.pathfindToPose(FieldLocations.reef4, SwerveConfig.pathConstraints);
                break;
            case 5:
                goToReefCommand = AutoBuilder.pathfindToPose(FieldLocations.reef5, SwerveConfig.pathConstraints);
                break;
            case 6:
                goToReefCommand = AutoBuilder.pathfindToPose(FieldLocations.reef6, SwerveConfig.pathConstraints);
                break;
            default:
                goToReefCommand = Commands.run(() -> {
                    System.out.println("Invalid reef number: " + reef);
                });
        }

        return new SequentialCommandGroup(goToReefCommand, Commands.waitSeconds(0.5));
    }

    /**
     * @return A command to get coral and go to all reefs, repeating the process.
     */
    public Command getCoralAndGoToAllReefsTest() {
        return new SequentialCommandGroup(
                getCoralFromStation(1),
                goToReef(4),
                getCoralFromStation(1),
                goToReef(5),
                getCoralFromStation(1),
                goToReef(6),
                getCoralFromStation(2),
                goToReef(1),
                getCoralFromStation(2),
                goToReef(2),
                getCoralFromStation(2),
                goToReef(3));
    }
}
