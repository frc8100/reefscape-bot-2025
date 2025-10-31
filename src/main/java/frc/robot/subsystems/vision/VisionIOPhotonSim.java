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

import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

/** IO implementation for vision sim using PhotonVision simulator. */
public class VisionIOPhotonSim extends VisionIOPhotonVision {

    private final PhotonCameraSim cameraSim;

    /**
     * The neural detector pipelines. Can be null if not used.
     */
    private final NeuralDetectorSimPipeline[] pipelines;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     * @param name - The name of the camera.
     * @param poseSupplier - Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonSim(
        String name,
        Transform3d robotToCamera,
        // Supplier<Pose2d> poseSupplier,
        RobotPoseAtTimestampSupplier robotPoseSupplier,
        SimCameraProperties cameraProperties,
        NeuralDetectorSimPipeline[] pipelines
    ) {
        super(name, robotToCamera, robotPoseSupplier);

        // Add sim camera
        cameraSim = new PhotonCameraSim(camera, cameraProperties, VisionConstants.aprilTagLayout);
        VisionSim.getVisionSim().addCamera(cameraSim, robotToCamera);

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

        Pose3d cameraPose = VisionSim.getVisionSim().getCameraPose(cameraSim, nextTimeMicroseconds);

        // Object detection
        cameraSim.canSeeTargetPose(new Pose3d(robotPoseSupplier.getRobotPoseAtTimestamp(nextEntryTimeMicrosecondsOpt.get() / 1e9)), null);
    }
}
