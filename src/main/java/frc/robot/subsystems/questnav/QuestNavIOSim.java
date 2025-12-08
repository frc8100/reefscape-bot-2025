package frc.robot.subsystems.questnav;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.Swerve;
import gg.questnav.questnav.PoseFrame;
import java.util.Random;
import java.util.function.Supplier;

/**
 * Simulates the QuestNav IO by providing pose data from the swerve subsystem.
 */
public class QuestNavIOSim implements QuestNavIO {

    // Noise parameters
    private static final double TRANSLATION_NOISE_STDDEV_METERS_PER_METERS_PER_SECOND = Centimeters.of(0.2).in(Meters);
    private static final double TRANSLATION_NOISE_STDDEV_METERS_BASE = Centimeters.of(0.1).in(Meters);
    private static final double Z_NOISE_STDDEV_METERS = Centimeters.of(0.2).in(Meters);

    private static final double ROTATION_NOISE_STDDEV_RADIANS_PER_METERS_PER_SECOND = Degrees.of(0.35).in(Radians);
    private static final double ROTATION_NOISE_STDDEV_RADIANS_BASE = Degrees.of(0.15).in(Radians);

    private final Random random = new Random();

    private final Supplier<Pose2d> simulatedPoseSupplier;
    private final Swerve swerveSubsystem;
    private int frameCounter = 1;

    public QuestNavIOSim(Swerve swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        simulatedPoseSupplier = swerveSubsystem::getActualPose;
    }

    /**
     * @return The simulated pose with added noise.
     */
    private Pose3d getSimulatedPose() {
        Pose2d pose2d = simulatedPoseSupplier.get();

        // Add some noise to the simulated pose
        double noiseX =
            random.nextGaussian() *
            (TRANSLATION_NOISE_STDDEV_METERS_BASE +
                TRANSLATION_NOISE_STDDEV_METERS_PER_METERS_PER_SECOND * swerveSubsystem.getVelocityMagnitudeAsDouble());
        double noiseY =
            random.nextGaussian() *
            (TRANSLATION_NOISE_STDDEV_METERS_BASE +
                TRANSLATION_NOISE_STDDEV_METERS_PER_METERS_PER_SECOND * swerveSubsystem.getVelocityMagnitudeAsDouble());
        double noiseZ = random.nextGaussian() * Z_NOISE_STDDEV_METERS;
        double noiseYaw =
            random.nextGaussian() *
            (ROTATION_NOISE_STDDEV_RADIANS_BASE +
                ROTATION_NOISE_STDDEV_RADIANS_PER_METERS_PER_SECOND * swerveSubsystem.getVelocityMagnitudeAsDouble());

        return new Pose3d(
            pose2d.getX() + noiseX,
            pose2d.getY() + noiseY,
            noiseZ,
            new Rotation3d(0, 0, pose2d.getRotation().getRadians() + noiseYaw)
        );
    }

    @Override
    public void updateInputs(QuestNavIOInputs inputs) {
        inputs.connected = true;
        inputs.isTracking = true;
        inputs.batteryPercent = 100;
        inputs.trackingLostCounter = 0;

        inputs.unreadPoseFrames = new PoseFrame[] {
            new PoseFrame(
                getSimulatedPose().transformBy(QuestNavSubsystem.ROBOT_TO_QUEST),
                Timer.getTimestamp(),
                Timer.getTimestamp(),
                frameCounter
            ),
        };

        frameCounter++;
    }
}
