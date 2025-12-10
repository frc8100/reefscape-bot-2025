package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N0;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.LinearSystem;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;

public class TrackedVisionTarget {

    // State and measurement standard deviations
    // TODO: Move to constants file
    public static final Matrix<N4, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 1.0, 1.0);
    public static final Matrix<N2, N1> measurementStdDevs = VecBuilder.fill(0.1, 0.1);

    private static final Matrix<N0, N1> emptyInput = new Matrix<>(N0.instance, N1.instance);

    /**
     * A constant-velocity LinearSystem for a 4-state ([x,y,vx,vy]) and 2-output ([x,y]) measurement.
     * The returned system has input dimension 0.
     */
    public static final LinearSystem<N4, N0, N2> visionTargetLinearSystem = new LinearSystem<>(
        // A matrix (4x4) - Continuous Time
        // Row 1: dx/dt = vx
        // Row 2: dy/dt = vy
        // Row 3: dvx/dt = 0 (Constant velocity assumption)
        // Row 4: dvy/dt = 0
        new Matrix<>(
            N4.instance,
            N4.instance,
            // @formatter:off
            new double[] {
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0
            }
            // @formatter:on
        ),
        // B is 4x0 (no inputs)
        new Matrix<>(N4.instance, N0.instance),
        // C maps state -> measurement (2x4)
        // Measure x and y directly.
        new Matrix<>(
            N2.instance,
            N4.instance,
            // @formatter:off
            new double[] {
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0
            }
            // @formatter:on
        ),
        // D is 2x0 (no feedthrough)
        new Matrix<>(N2.instance, N0.instance)
    );

    /**
     * The type of game piece the target represents.
     */
    public final GamePieceObservationType type;

    /**
     * The time the target was created.
     */
    public final double creationTimeSeconds;

    /**
     * The time the target was last seen.
     */
    public double lastSeenTimeSeconds;

    /**
     * The number of hits for this target.
     */
    public int hits = 0;

    /**
     * The number of misses for this target.
     */
    public int misses = 0;

    /**
     * The latest pose of the target.
     */
    public Pose3d latestPose;

    /**
     * A Kalman filter for estimating the pose of the vision target.
     * The state vector is [x, y, vx, vy].
     * The output vector is [x, y].
     */
    public KalmanFilter<N4, N0, N2> kalmanFilter = new KalmanFilter<>(
        N4.instance,
        N2.instance,
        visionTargetLinearSystem,
        stateStdDevs,
        measurementStdDevs,
        Constants.LOOP_PERIOD_SECONDS
    );

    public TrackedVisionTarget(GamePieceObservationType type, double creationTimeSeconds, Pose3d initialPose) {
        this.type = type;
        this.creationTimeSeconds = creationTimeSeconds;
        this.latestPose = initialPose;

        // Seed filter
        // Set x and y to the measurement
        // Leave vx and vy at 0
        kalmanFilter.setXhat(0, initialPose.getX());
        kalmanFilter.setXhat(1, initialPose.getY());
    }

    private final Matrix<N2, N1> cachedMeasurement = new Matrix<>(N2.instance, N1.instance);

    /**
     * Updates the pose of the target.
     * @param newPose - The new pose of the target.
     */
    public void updatePose(Pose3d newPose) {
        this.latestPose = newPose;
        this.hits++;
        // Reset misses on a successful update
        this.misses = 0;

        // Update the Kalman filter with the new measurement
        cachedMeasurement.set(0, 0, newPose.getX());
        cachedMeasurement.set(1, 0, newPose.getY());
        kalmanFilter.correct(emptyInput, cachedMeasurement);
    }

    /**
     * Predicts the next state of the target.
     * Should be called once per update cycle.
     */
    public void update() {
        kalmanFilter.predict(emptyInput, Constants.LOOP_PERIOD_SECONDS);
    }

    public Pose2d getEstimatedPose2d() {
        Matrix<N4, N1> state = kalmanFilter.getXhat();
        return new Pose2d(state.get(0, 0), state.get(1, 0), latestPose.getRotation().toRotation2d());
    }

    /**
     * @return True if the target should be deleted due to too many misses.
     */
    public boolean shouldDelete() {
        return misses > 5;
    }
}
