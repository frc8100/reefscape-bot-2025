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
import java.util.LinkedList;
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

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        Optional<Long> nextEntryTimeMicrosecondsOpt = cameraSim.consumeNextEntryTime();
        if (!nextEntryTimeMicrosecondsOpt.isPresent()) {
            // No new data
            return;
        }
        long nextTimeMicroseconds = nextEntryTimeMicrosecondsOpt.get();

        Pose3d cameraPose = new Pose3d(robotPoseSupplier.get()).transformBy(robotToCamera);

        // Object detection
        List<GamePieceObservation> gamePieceObservations = new LinkedList<>();

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

        inputs.gamePieceObservations = gamePieceObservations.toArray(new GamePieceObservation[0]);
    }
}
