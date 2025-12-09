package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.Matrix;
import frc.robot.subsystems.vision.VisionIO.GamePieceObservation;
import frc.robot.subsystems.vision.objectdetection.GamePiecePoseEstimator.DetectionAssociatedWithTrackedTarget;
import java.util.List;

/**
 * Implementation of the Hungarian Algorithm for assigning detected objects to tracked targets.
 */
public class HungarianAlgorithm {

    /**
     * Produce a cost matrix where rows are detections and columns are tracked targets.
     */
    private Matrix<?, ?> getCostMatrix() {
        // placeholder
        return null;
    }

    private final List<DetectionAssociatedWithTrackedTarget> cachedAssignments = List.of();

    public List<DetectionAssociatedWithTrackedTarget> assignDetectionsToTargets(
        List<GamePieceObservation> detections,
        List<TrackedVisionTarget> trackedTargets
    ) {
        cachedAssignments.clear();

        if (detections.isEmpty() || trackedTargets.isEmpty()) {
            return cachedAssignments;
        }

        // TODO: implement algorithm
        return cachedAssignments;
    }
}
