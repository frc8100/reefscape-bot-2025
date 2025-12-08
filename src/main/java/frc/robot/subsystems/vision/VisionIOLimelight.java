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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.RobotController;
import frc.util.LimelightHelpers;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {

    /**
     * The Limelight instance for this Limelight. Gets data from NetworkTables.
     */
    // private final Limelight limelight;

    /**
     * The name of this Limelight. Used for NetworkTables.
     */
    private final String name;

    /**
     * Supplier for the current estimated rotation, used for MegaTag 2.
     * See {@link frc.util.LimelightHelpers#SetRobotOrientation} for format.
     * Should be a double[6] array: {yaw, yawRate, pitch, pitchRate, roll, rollRate} in degrees and degrees per second.
     */
    private final Supplier<double[]> rotationSupplier;
    private final DoubleArrayPublisher orientationPublisher;

    private final DoubleSubscriber latencySubscriber;
    private final DoubleArraySubscriber megatag1Subscriber;
    private final DoubleArraySubscriber megatag2Subscriber;

    private final List<PoseObservation> poseObservations = new ArrayList<>();
    private final Set<Integer> tagIds = new HashSet<>();

    /**
     * Creates a new VisionIOLimelight.
     * @param name - The configured name of the Limelight.
     * @param rotationSupplier - Supplier for the current estimated rotation, used for MegaTag 2. See {@link #rotationSupplier} for details.
     */
    public VisionIOLimelight(String name, Supplier<double[]> rotationSupplier) {
        this.name = name;
        this.rotationSupplier = rotationSupplier;

        var table = NetworkTableInstance.getDefault().getTable(name);
        orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    }

    /**
     * Reads a pose observation from a raw NetworkTables sample.
     * @param rawSample - The raw sample double array from NetworkTables.
     * @param type - The type of pose observation.
     */
    private void readPoseObservation(TimestampedDoubleArray rawSample, PoseObservationType type) {
        if (rawSample.value.length == 0) return;

        // Add tag IDs to set
        for (int i = 11; i < rawSample.value.length; i += 7) {
            tagIds.add((int) rawSample.value[i]);
        }

        poseObservations.add(
            new PoseObservation(
                // Timestamp, based on server timestamp of publish and latency
                rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
                // 3D pose estimate
                parsePose(rawSample.value),
                // Ambiguity or zeroed for MegaTag 2
                type == PoseObservationType.MEGATAG_1 && rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,
                // Tag count
                (int) rawSample.value[7],
                // Average tag distance
                rawSample.value[9],
                // Observation type
                type
            )
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected = RobotController.getFPGATime() - latencySubscriber.getLastChange() < 0.25;

        // Update orientation for MegaTag 2
        orientationPublisher.accept(rotationSupplier.get());
        // Increases network traffic but recommended by Limelight to flush after publishing
        NetworkTableInstance.getDefault().flush();

        // Read new pose observations from NetworkTables
        for (TimestampedDoubleArray rawSample : megatag1Subscriber.readQueue()) {
            readPoseObservation(rawSample, PoseObservationType.MEGATAG_1);
        }
        for (TimestampedDoubleArray rawSample : megatag2Subscriber.readQueue()) {
            readPoseObservation(rawSample, PoseObservationType.MEGATAG_2);
        }

        // Save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }
        poseObservations.clear();

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
        tagIds.clear();
    }

    /**
     * Parses the 3D pose from a Limelight botpose array.
     */
    private static Pose3d parsePose(double[] rawLLArray) {
        return new Pose3d(
            rawLLArray[0],
            rawLLArray[1],
            rawLLArray[2],
            new Rotation3d(
                Units.degreesToRadians(rawLLArray[3]),
                Units.degreesToRadians(rawLLArray[4]),
                Units.degreesToRadians(rawLLArray[5])
            )
        );
    }
}
