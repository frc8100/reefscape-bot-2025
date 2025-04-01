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

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.FieldConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * The vision subsystem. This subsystem processes vision data and sends it to the robot code.
 */
public class Vision extends SubsystemBase {

    /**
     * The consumer for vision data. Usually a method to add a measurement to the pose estimator.
     */
    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs
        );
    }

    private final VisionConsumer consumer;

    /**
     * A list of all vision IOs. Each IO corresponds to a camera.
     */
    private final VisionIO[] io;

    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert(
                "Vision camera " + Integer.toString(i) + " is disconnected.",
                AlertType.kWarning
            );
        }
    }

    /**
     * @return A list of all Photon pipeline results from all cameras.
     * If photon vision is not used, this method returns an empty list.
     */
    // public List<PhotonPipelineResult> getPhotonPipelineResults() {
    //     List<PhotonPipelineResult> results = new LinkedList<>();

    //     // For each photon vision IO, add all unread results
    //     for (VisionIO photonVisionIO : io) {
    //         results.addAll(photonVisionIO.getPhotonPipelineResults());
    //     }

    //     return results;
    // }

    /**
     * @return The latest target from the specified camera, if it exists.
     */
    public Optional<PhotonPipelineResult> getLatestTargetFromCamera(int cameraIndex) {
        // List<PhotonPipelineResult> pipelineResults = io[cameraIndex].getPhotonPipelineResults();

        // // Return the latest target if it exists
        // if (pipelineResults.size() > 0) {
        //     return Optional.of(pipelineResults.get(pipelineResults.size() - 1));
        // }

        // // Return empty if no targets
        // return Optional.empty();

        return Optional.of(io[cameraIndex].getLatestPipelineResult());
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    @Override
    public void periodic() {
        // Update inputs
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (PoseObservation observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose =
                    observation.tagCount() == 0 || // Must have at least one tag
                    (observation.tagCount() == 1 && observation.ambiguity() > MAX_AMBIGUITY) || // Cannot be high ambiguity
                    Math.abs(observation.pose().getZ()) > MAX_Z_ERROR || // Must have realistic Z coordinate
                    // Must be within the field boundaries
                    observation.pose().getX() <
                    0.0 ||
                    observation.pose().getX() > FieldConstants.fieldLength.in(Meters) ||
                    observation.pose().getY() < 0.0 ||
                    observation.pose().getY() > FieldConstants.fieldWidth.in(Meters);

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
                double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= LINEAR_STD_DEV_MEGATAG2_FACTOR;
                    angularStdDev *= ANGULAR_STD_DEV_MEGATAG2_FACTOR;
                }
                if (cameraIndex < CAMERA_STD_DEV_FACTORS.length) {
                    linearStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
                    angularStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
                }

                // Send vision observation
                consumer.accept(
                    observation.pose().toPose2d(),
                    observation.timestamp(),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                );
            }

            // Log camera datadata
            // Logger.recordOutput(
            //     "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
            //     tagPoses.toArray(new Pose3d[tagPoses.size()])
            // );
            // Logger.recordOutput(
            //     "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
            //     robotPoses.toArray(new Pose3d[robotPoses.size()])
            // );
            // Logger.recordOutput(
            //     "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
            //     robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()])
            // );
            // Logger.recordOutput(
            //     "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
            //     robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()])
            // );
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        // Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        // Logger.recordOutput(
        //     "Vision/Summary/RobotPosesAccepted",
        //     allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()])
        // );
        // Logger.recordOutput(
        //     "Vision/Summary/RobotPosesRejected",
        //     allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()])
        // );

        // debug
    }

    /**
     * For debugging purposes, returns the target relative to camera 0.
     *
     * @return The target relative to camera 0, or a 0 translation if no target is found.
     */
    public Translation2d debugGetTargetRelativeToCamera0(double tagHeightMeters) {
        var latestObservation = getLatestTargetFromCamera(0);
        if (latestObservation.isEmpty()) {
            // Return a zero translation if no target is found
            return new Translation2d(0, 0);
        }

        // Calculate tag position
        var latestObservationTarget = latestObservation.get().getBestTarget();

        double targetYaw = latestObservationTarget.getYaw();
        double targetDistance = PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.TRANSFORM_TO_CAMERA_0.getZ(),
            tagHeightMeters,
            VisionConstants.TRANSFORM_TO_CAMERA_0.getRotation().getZ(),
            Units.degreesToRadians(latestObservationTarget.getPitch())
        );

        Translation2d targetRelativeToCamera = PhotonUtils.estimateCameraToTargetTranslation(
            targetDistance,
            Rotation2d.fromDegrees(targetYaw)
        );

        return targetRelativeToCamera;
    }
}
