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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.util.VisionUtil;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * IO implementation for real PhotonVision hardware.
 */
public class VisionIOPhotonVision implements VisionIO {

    /**
     * The PhotonCamera instance for this camera.
     */
    protected final PhotonCamera camera;

    /**
     * The transform from the robot to the camera.
     */
    protected final Transform3d robotToCamera;

    // protected final RobotPoseAtTimestampSupplier robotPoseSupplier;

    /**
     * A list of all Photon pipeline results from this camera.
     * If photon vision is not used, this is an empty list.
     */
    // private List<PhotonPipelineResult> photonPipelineResults = new LinkedList<>();

    /**
     * Creates a new VisionIOPhotonVision.
     * @param name The configured name of the camera.
     * @param rotationSupplier The 3D position of the camera relative to the robot.
     */
    public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
        // this.robotPoseSupplier = robotPoseSupplier;
    }

    // private Optional<GamePieceObservation> convertGamePieceObservation(PhotonTrackedTarget target, double timestampSeconds) {
    //     Optional<Pose2d> robotPoseOpt = robotPoseSupplier.getRobotPoseAtTimestamp(timestampSeconds);

    //     if (robotPoseOpt.isEmpty()) {
    //         return Optional.empty();
    //     }

    //     return Optional.of(new GamePieceObservation(
    //         timestampSeconds,
    //         VisionUtil.estimateTargetPose3d(
    //             robotPoseOpt.get(),
    //             robotToCamera,
    //             Degrees.of(target.getYaw()),
    //             Degrees.of(target.getPitch()),
    //             Meters.of(0)
    //         ),
    //         target.objDetectConf,
    //         GamePieceObservationType.fromClassID(target.objDetectId)
    //     ));
    // }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        // Read new camera observations
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();

        // Add results to inputs
        // TODO: Optimize memory usage, this creates a new list every time
        List<PhotonPipelineResult> photonPipelineResults = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : photonPipelineResults) {
            // Add pose observation
            if (result.multitagResult.isPresent()) {
                // Multitag result
                var multitagResult = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                // Calculate average tag distance
                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                // Add tag IDs
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                // Add observation
                poseObservations.add(
                    new PoseObservation(
                        result.getTimestampSeconds(), // Timestamp
                        robotPose, // 3D pose estimate
                        multitagResult.estimatedPose.ambiguity, // Ambiguity
                        multitagResult.fiducialIDsUsed.size(), // Tag count
                        totalTagDistance / result.targets.size(), // Average tag distance
                        PoseObservationType.PHOTONVISION
                    )
                ); // Observation type
            } else if (!result.targets.isEmpty()) { // Single tag result
                var target = result.targets.get(0);

                // Calculate robot pose
                var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(
                        tagPose.get().getTranslation(),
                        tagPose.get().getRotation()
                    );
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    // Add tag ID
                    tagIds.add((short) target.fiducialId);

                    // Add observation
                    poseObservations.add(
                        new PoseObservation(
                            result.getTimestampSeconds(), // Timestamp
                            robotPose, // 3D pose estimate
                            target.poseAmbiguity, // Ambiguity
                            1, // Tag count
                            cameraToTarget.getTranslation().getNorm(), // Average tag distance
                            PoseObservationType.PHOTONVISION
                        )
                    ); // Observation type
                }
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }
}
