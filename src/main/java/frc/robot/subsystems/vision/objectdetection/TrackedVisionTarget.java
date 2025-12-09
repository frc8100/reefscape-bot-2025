package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.PoseEstimator3d;
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

    public KalmanFilter kalmanFilter;

    public TrackedVisionTarget(GamePieceObservationType type, double creationTimeSeconds) {
        this.type = type;
        this.creationTimeSeconds = creationTimeSeconds;
        this.hits = 0;
        this.misses = 0;
    }
}
