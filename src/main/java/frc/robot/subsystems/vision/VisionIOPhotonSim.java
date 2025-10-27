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
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonSim extends VisionIOPhotonVision {

    private final PhotonCameraSim cameraSim;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     * @param name - The name of the camera.
     * @param poseSupplier - Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonSim(
        String name,
        Transform3d robotToCamera,
        Supplier<Pose2d> poseSupplier,
        SimCameraProperties cameraProperties
    ) {
        super(name, robotToCamera);

        // Add sim camera
        cameraSim = new PhotonCameraSim(camera, cameraProperties, VisionConstants.aprilTagLayout);
        VisionSim.getVisionSim().addCamera(cameraSim, robotToCamera);
    }
}
