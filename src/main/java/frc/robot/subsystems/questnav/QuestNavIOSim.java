package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.SwerveDrive;
import gg.questnav.questnav.PoseFrame;
import java.util.function.Supplier;

/**
 * Simulates the QuestNav IO by providing pose data from the SwerveDrive subsystem.
 */
public class QuestNavIOSim implements QuestNavIO {

    private Supplier<Pose2d> simulatedPoseSupplier;
    private int frameCounter = 1;

    public QuestNavIOSim(SwerveDrive swerveSubsystem) {
        simulatedPoseSupplier = swerveSubsystem::getActualPose;
    }

    @Override
    public void updateInputs(QuestNavIOInputs inputs) {
        inputs.connected = true;
        inputs.isTracking = true;
        inputs.batteryPercent = 100;
        inputs.trackingLostCounter = 0;

        inputs.unreadPoseFrames = new PoseFrame[] {
            new PoseFrame(
                simulatedPoseSupplier.get().transformBy(QuestNavSubsystem.ROBOT_TO_QUEST),
                Timer.getTimestamp(),
                Timer.getTimestamp(),
                frameCounter
            ),
        };

        frameCounter++;
    }
}
