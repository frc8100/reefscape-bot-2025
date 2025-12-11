package frc.robot.subsystems.vision.objectdetection;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import frc.robot.subsystems.vision.VisionIO.GamePieceObservation;
import frc.robot.subsystems.vision.objectdetection.GamePiecePoseEstimator.DetectionAssociatedWithTrackedTarget;
import java.util.ArrayList;
import java.util.List;

/**
 * Implementation of the Hungarian Algorithm for assigning detected objects to tracked targets.
 */
public class HungarianAlgorithm {

    /**
     * Maximum cost allowed for greedy assignment in meters.
     */
    private static final double MAX_COST_ALLOWED_FOR_GREEDY_ASSIGNMENT_METERS = Inches.of(4).in(Meters);

    /**
     * When the cost of assigning a detection to a tracked target is below this threshold, instantly assign them without checking other options.
     */
    private static final double INSTANT_ASSIGNMENT_COST_THRESHOLD_METERS = Inches.of(2).in(Meters);

    /**
     * Produce a cost matrix where rows are detections and columns are tracked targets.
     */
    // private Matrix<?, ?> getCostMatrix(
    //     List<GamePieceObservation> detections,
    //     List<TrackedVisionTarget> trackedTargets
    // ) {
    //     return null;
    // }

    /**
     * Calculate the cost of assigning a detection to a tracked target.
     * The cost is the Euclidean distance between the detection pose and the tracked target's estimated pose.
     * @param detection - The game piece observation detection.
     * @param trackedTarget - The tracked vision target.
     * @return The cost of the assignment.
     */
    private double getCost(GamePieceObservation detection, TrackedVisionTarget trackedTarget) {
        return detection.pose().getTranslation().getDistance(trackedTarget.getEstimatedPose().getTranslation());
    }

    private final List<DetectionAssociatedWithTrackedTarget> cachedAssignments = new ArrayList<>();

    /**
     * Assign detections to tracked targets using a greedy algorithm based on cost.
     * @param detections - The list of game piece observations.
     * @param trackedTargets - The list of tracked vision targets.
     * @return A list of assignments of detections to tracked targets.
     */
    public List<DetectionAssociatedWithTrackedTarget> assignDetectionsToTargets(
        List<GamePieceObservation> detections,
        List<TrackedVisionTarget> trackedTargets
    ) {
        cachedAssignments.clear();

        // No assignments possible
        if (detections.isEmpty() || trackedTargets.isEmpty()) {
            return cachedAssignments;
        }

        // For every tracked target and detection, reset assignment flags
        for (TrackedVisionTarget target : trackedTargets) {
            target.hasBeenAssignedThisFrame = false;
        }

        // Greedy algorithm
        for (GamePieceObservation detection : detections) {
            TrackedVisionTarget bestTarget = null;
            double bestCost = Double.MAX_VALUE;

            for (TrackedVisionTarget trackedTarget : trackedTargets) {
                // Skip already assigned targets
                if (trackedTarget.hasBeenAssignedThisFrame) {
                    continue;
                }

                double cost = getCost(detection, trackedTarget);
                if (cost < bestCost) {
                    bestCost = cost;
                    bestTarget = trackedTarget;
                }

                // If the cost is below the instant assignment threshold, we can assign it immediately.
                if (bestCost <= INSTANT_ASSIGNMENT_COST_THRESHOLD_METERS) {
                    break;
                }
            }

            if (bestTarget != null && bestCost <= MAX_COST_ALLOWED_FOR_GREEDY_ASSIGNMENT_METERS) {
                cachedAssignments.add(new DetectionAssociatedWithTrackedTarget(detection, bestTarget));
            }
        }

        return cachedAssignments;
    }
}
