package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavIOReal implements QuestNavIO {

    private QuestNav questNav = new QuestNav();

    @Override
    public void updateInputs(QuestNavIOInputs inputs) {
        // Required to call periodically to update internal state
        questNav.commandPeriodic();

        inputs.connected = questNav.isConnected();
        inputs.isTracking = questNav.isTracking();
        inputs.batteryPercent = questNav.getBatteryPercent().orElse(-1);
        inputs.trackingLostCounter = questNav.getTrackingLostCounter().orElse(-1);

        if (inputs.isTracking) {
            inputs.unreadPoseFrames = questNav.getAllUnreadPoseFrames();
        } else {
            inputs.unreadPoseFrames = new PoseFrame[0];
        }
    }

    @Override
    public void setPose(Pose2d pose) {
        // Transform the pose to the headset's frame of reference
        pose = pose.transformBy(QuestNavSubsystem.ROBOT_TO_QUEST);

        questNav.setPose(pose);
    }
}
