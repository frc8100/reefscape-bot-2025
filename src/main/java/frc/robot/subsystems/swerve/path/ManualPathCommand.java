package frc.robot.subsystems.swerve.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

@Deprecated
public class ManualPathCommand extends Command {
    private final SwerveDrive swerveSubsystem;
    private final Pose2d targetPose;

    private Command followCommand = null;

    public ManualPathCommand(SwerveDrive swerveSubsystem, Pose2d targetPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetPose = targetPose;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        Command followCommandToSchedule =
                swerveSubsystem.getPathfindingInstance().generateGoToPoseCommand(targetPose);

        // followCommandToSchedule.finallyDo(() -> {
        //     System.out.println("Finished following path");
        //     this.end(false);
        // });

        followCommandToSchedule.schedule();

        // Set the follow command to the generated command
        followCommand = followCommandToSchedule;
    }

    @Override
    public void end(boolean interrupted) {
        // swerveSubsystem.stop();
        if (followCommand != null) {
            System.out.println("Cancelling follow command");

            followCommand.cancel();
        }
    }
}
