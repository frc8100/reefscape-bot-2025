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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Vision constants. Some must be configured in the web UI.
 */
public class VisionConstants {
    private VisionConstants() {}

    /**
     * The AprilTag field layout. Note: see TU 12
     */
    public static final AprilTagFieldLayout aprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    /**
     * Camera names, must match names configured on coprocessor
     */
    public static final String CAMERA_0_NAME = "limelight";

    /**
     * Robot to camera 0 transform
     * (Not used by Limelight, configure in web UI instead)
     */
    public static final Transform3d TRANSFORM_TO_CAMERA_0 =
            new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));

    // Basic filtering thresholds
    public static final double MAX_AMBIGUITY = 0.3;
    public static final double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
    public static final double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static final double[] CAMERA_STD_DEV_FACTORS = new double[] {
        1.0, // Camera 0
    };

    // Multipliers to apply for MegaTag 2 observations
    public static final double LINEAR_STD_DEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
    public static final double ANGULAR_STD_DEV_MEGATAG2_FACTOR = Double.POSITIVE_INFINITY; // No rotation data available
}
