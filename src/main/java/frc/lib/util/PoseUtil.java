package frc.lib.util;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.math.GeometryUtils;
import java.util.Optional;

/**
 * A utility class for working with poses.
 * Note: Use {@link FlippingUtil instead}.
 */
public final class PoseUtil {

    private PoseUtil() {}

    /**
     * Checks if two poses are within a specified tolerance of each other.
     * @param pose1 - The first pose.
     * @param pose2 - The second pose.
     * @param tolerance - The tolerance to check within. Includes x, y, and rotation.
     * @return true if the poses are within the specified tolerance, false otherwise.
     */
    public static boolean isNear(Pose2d pose1, Pose2d pose2, double tolerance) {
        return (
            Math.abs(pose1.getX() - pose2.getX()) < tolerance &&
            Math.abs(pose1.getY() - pose2.getY()) < tolerance &&
            Math.abs(pose1.getRotation().getRadians() - pose2.getRotation().getRadians()) < tolerance
        );
    }

    public static double applyX(double x) {
        return shouldFlip() ? FieldConstants.fieldLength.in(Meters) - x : x;
    }

    public static double applyY(double y) {
        return shouldFlip() ? FieldConstants.fieldLength.in(Meters) - y : y;
    }

    public static Translation2d apply(Translation2d translation) {
        return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
    }

    public static Rotation2d apply(Rotation2d rotation) {
        return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
    }

    public static Pose2d apply(Pose2d pose) {
        return shouldFlip() ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation())) : pose;
    }

    public static Translation3d apply(Translation3d translation) {
        return new Translation3d(applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
    }

    public static Rotation3d apply(Rotation3d rotation) {
        return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
    }

    public static Pose3d apply(Pose3d pose) {
        return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
    }

    /**
     * Checks if the robot should flip its pose based on the current alliance color.
     * @return true if the robot is on the red alliance, false otherwise.
     */
    public static boolean shouldFlip() {
        return (
            DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        );
    }
}
