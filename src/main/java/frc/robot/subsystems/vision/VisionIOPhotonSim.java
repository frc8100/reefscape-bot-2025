// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionSim.NeuralDetectorSimPipeline;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;

/**
 * IO implementation for vision sim using PhotonVision simulator.
 * Currently only supports object detection.
 */
public class VisionIOPhotonSim extends VisionIOPhotonVision {

    private final PhotonCameraSim cameraSim;

    /**
     * The neural detector pipelines. Can be null if not used.
     */
    private final NeuralDetectorSimPipeline[] pipelines;

    private final Supplier<Pose2d> robotPoseSupplier;
    private final List<GamePieceObservation> gamePieceObservations = new ArrayList<>();

    /**
     * Creates a new VisionIOPhotonVisionSim.
     * @param name - The name of the camera.
     * @param poseSupplier - Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonSim(
        String name,
        Transform3d robotToCamera,
        SimCameraProperties cameraProperties,
        Supplier<Pose2d> robotPoseSupplier,
        NeuralDetectorSimPipeline[] pipelines
    ) {
        super(name, robotToCamera);
        // Add sim camera
        cameraSim = new PhotonCameraSim(camera, cameraProperties, VisionConstants.aprilTagLayout);
        VisionSim.getVisionSim().addCamera(cameraSim, robotToCamera);

        this.robotPoseSupplier = robotPoseSupplier;
        this.pipelines = pipelines;
    }

    private static Field nextNTEntryTimeField = null;

    /**
     * @return The Field object for the {@link PhotonCameraSim#nextNTEntryTime} field. Can be null if an error occurred.
     */
    private static Field getNextNTEntryTimeField() {
        if (nextNTEntryTimeField == null) {
            try {
                // Use reflection to access private field
                nextNTEntryTimeField = PhotonCameraSim.class.getDeclaredField("nextNTEntryTime");
                nextNTEntryTimeField.setAccessible(true); // NOSONAR
            } catch (NoSuchFieldException e) {
                e.printStackTrace();
            }
        }

        return nextNTEntryTimeField;
    }

    /**
     * Gets the next entry time without consuming it.
     * See {@link PhotonCameraSim#consumeNextEntryTime()}.
     * @return The next entry time in microseconds, or empty if an error occurred or no entry time is available.
     */
    private Optional<Long> getNextEntryTimeNoConsume() {
        try {
            Field field = getNextNTEntryTimeField();
            if (field == null) {
                return Optional.empty();
            }

            // Store old value
            long oldNextNTEntryTime = (long) field.get(cameraSim);

            Optional<Long> nextEntryTimeMicrosecondsOpt = cameraSim.consumeNextEntryTime();

            // Restore old value
            field.set(cameraSim, oldNextNTEntryTime); // NOSONAR

            return nextEntryTimeMicrosecondsOpt;
        } catch (IllegalAccessException e) {
            e.printStackTrace();
            return Optional.empty();
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Optional<Long> nextEntryTimeMicrosecondsOpt = getNextEntryTimeNoConsume();
        Optional<Long> nextEntryTimeMicrosecondsOpt = cameraSim.consumeNextEntryTime();

        if (!nextEntryTimeMicrosecondsOpt.isPresent()) {
            // No new data
            return;
        }
        long nextTimeMicroseconds = nextEntryTimeMicrosecondsOpt.get();

        Pose3d cameraPose = new Pose3d(robotPoseSupplier.get()).transformBy(robotToCamera);

        // Object detection

        // For each pipeline, get potential targets and see if they are visible
        for (NeuralDetectorSimPipeline pipeline : pipelines) {
            List<VisionTargetSim> potentialTargets = VisionSim.getVisionTargetSimFromNeuralPipeline(pipeline);

            for (VisionTargetSim target : potentialTargets) {
                if (cameraSim.canSeeTargetPose(cameraPose, target)) {
                    // Visible, add observation
                    gamePieceObservations.add(
                        new GamePieceObservation(nextTimeMicroseconds / 1e6, target.getPose(), 0.0, pipeline.type())
                    );
                }
            }
        }

        inputs.gamePieceObservations = gamePieceObservations.toArray(
            new GamePieceObservation[gamePieceObservations.size()]
        );
        gamePieceObservations.clear();
        // Update vision sim with current robot pose
        // TODO: Make this only do once with multiple cameras (this updates all cameras)
        // VisionSim.getVisionSim().update(robotPoseSupplier.get());

        // super.updateInputs(inputs);
    }
}
