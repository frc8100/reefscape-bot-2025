package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;

public class TrackedVisionTarget {

    /**
     * The type of game piece the target represents.
     */
    public final GamePieceObservationType type;

    /**
     * The time the target was created.
     */
    public final double creationTimeSeconds;

    /**
     * The number of hits for this target.
     */
    public int hits;

    /**
     * The number of misses for this target.
     */
    public int misses;

    /**
     * The latest pose of the target.
     */
    public Pose3d latestPose;

    // public KalmanFilter kalmanFilter;

    public TrackedVisionTarget(GamePieceObservationType type, double creationTimeSeconds, Pose3d initialPose) {
        this.type = type;
        this.creationTimeSeconds = creationTimeSeconds;
        this.latestPose = initialPose;
        this.hits = 0;
        this.misses = 0;
    }

    /**
     * Updates the pose of the target.
     * @param newPose - The new pose of the target.
     */
    public void updatePose(Pose3d newPose) {
        this.latestPose = newPose;
        this.hits++;
        // Reset misses on a successful update
        this.misses = 0;
    }

    /**
     * @return True if the target should be deleted due to too many misses.
     */
    public boolean shouldDelete() {
        return misses > 5;
    }
}
