package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;
import frc.robot.subsystems.vision.VisionIO.GamePieceObservation;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Implements a Kalman filter-based pose estimator for game pieces.
 */
public class GamePiecePoseEstimator {

    /**
     * A detection associated with a tracked target.
     * @param observation - The game piece observation.
     * @param trackedTarget - The tracked vision target.s
     */
    public record DetectionAssociatedWithTrackedTarget(
        GamePieceObservation observation,
        TrackedVisionTarget trackedTarget
    ) {}

    private final HungarianAlgorithm hungarianAlgorithm = new HungarianAlgorithm();

    /**
     * The tracked vision targets by type.
     */
    private final Map<GamePieceObservationType, List<TrackedVisionTarget>> trackedTargets = new EnumMap<>(
        GamePieceObservationType.class
    );

    /**
     * Constructs a new GamePiecePoseEstimator.
     */
    public GamePiecePoseEstimator() {
        // Initialize the map with empty lists for each observation type
        for (GamePieceObservationType type : GamePieceObservationType.values()) {
            trackedTargets.put(type, new ArrayList<>());
        }
    }

    /**
     * Gets the latest observed game piece poses for the given type.
     * @param type - The game piece observation type.
     * @return A list of the latest observed game piece poses for the given type. If no poses have been observed, returns an empty list.
     */
    public List<Pose3d> getLatestGamePiecePoses(GamePieceObservationType type) {
        return trackedTargets.get(type).stream().map(target -> target.getEstimatedPose()).toList();
    }

    /**
     * Gets the latest observed game piece poses as 2D poses for the given type.
     * @param type - The game piece observation type.
     * @return A list of the latest observed game piece poses as 2D poses for the given type. If no poses have been observed, returns an empty list.
     */
    public List<Pose2d> getLatestGamePiecePosesAs2d(GamePieceObservationType type) {
        return trackedTargets.get(type).stream().map(target -> target.getEstimatedPose().toPose2d()).toList();
    }

    /**
     * Gets the nearest observed game piece pose of the given type to the reference pose.
     * @param type - The game piece observation type.
     * @param referencePoseSupplier - A supplier that provides the reference pose.
     * @return An Optional containing the nearest observed game piece pose of the given type to the reference pose, or an empty Optional if no poses have been observed.
     */
    public Optional<Pose3d> getNearestGamePiecePose(
        GamePieceObservationType type,
        Supplier<Pose2d> referencePoseSupplier
    ) {
        List<Pose3d> poses = getLatestGamePiecePoses(type);

        // Return empty if no poses are available
        if (poses.isEmpty()) {
            return Optional.empty();
        }

        Pose2d referencePose = referencePoseSupplier.get();

        return Optional.of(
            Collections.min(
                poses,
                Comparator.comparingDouble(pose ->
                    pose.toPose2d().getTranslation().getDistance(referencePose.getTranslation())
                )
            )
        );
    }

    /**
     * Updates the estimator with new game piece observations.
     * @param observations - An array of game piece observations.
     */
    public void updateWithObservations(GamePieceObservation[] observations) {
        // Skip empty observations
        if (observations.length == 0) {
            return;
        }
        // updateWithAssociatedTargets(hungarianAlgorithm.assignDetectionsToTargets(
        //     List.of(observations),
        //     trackedTargets.values().stream().flatMap(List::stream).toList()
        // ));
    }

    private void updateWithAssociatedTargets(
        List<DetectionAssociatedWithTrackedTarget> associatedDetections,
        List<GamePieceObservation> unassociatedDetections
    ) {
        // Update associated targets
        for (DetectionAssociatedWithTrackedTarget associatedDetection : associatedDetections) {
            GamePieceObservation observation = associatedDetection.observation;
            TrackedVisionTarget trackedTarget = associatedDetection.trackedTarget;

            // Update the tracked target with the new observation pose
            trackedTarget.updatePose(observation.pose());
        }

        // Create new tracked targets for unassociated detections
        for (GamePieceObservation observation : unassociatedDetections) {
            TrackedVisionTarget newTarget = new TrackedVisionTarget(
                observation.type(),
                observation.timestampSeconds(),
                observation.pose()
            );

            trackedTargets.get(observation.type()).add(newTarget);
        }
    }

    /**
     * Processes the observations to update hits and misses for tracked targets.
     * Should be called once per update cycle after observations have been processed.
     */
    public void processObservations() {
        for (List<TrackedVisionTarget> targets : trackedTargets.values()) {
            // Update each target miss
            for (TrackedVisionTarget target : targets) {
                if (!target.hasBeenHitThisFrame) {
                    target.misses++;
                }
            }

            // Remove targets that should be deleted
            targets.removeIf(TrackedVisionTarget::shouldDelete);

            // Update each target
            for (TrackedVisionTarget target : targets) {
                target.hasBeenHitThisFrame = false;

                // Update target kalman filter
                target.update();
            }
        }
    }

    /**
     * Logs the latest observed game piece poses by type.
     */
    public void logGamePiecePoses() {
        // For each game piece type, log the latest observed poses
        for (Map.Entry<GamePieceObservationType, List<TrackedVisionTarget>> entry : trackedTargets.entrySet()) {
            GamePieceObservationType type = entry.getKey();
            List<TrackedVisionTarget> targets = entry.getValue();

            Logger.recordOutput(
                "Vision/GamePieces/" + type.className + "/LatestPoses",
                // targets.stream().map(target -> target.latestPose).toArray(Pose3d[]::new)
                targets.stream().map(target -> target.getEstimatedPose()).toArray(Pose3d[]::new)
            );
        }
    }
}
