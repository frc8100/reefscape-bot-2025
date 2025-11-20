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

    public final double creationTimeSeconds;

    public int hits;

    public int misses;

    // public KalmanFilter kalmanFilter;

    public TrackedVisionTarget(GamePieceObservationType type) {
        this.type = type;
        this.creationTimeSeconds = Timer.getFPGATimestamp();
        this.hits = 0;
        this.misses = 0;
    }
}
