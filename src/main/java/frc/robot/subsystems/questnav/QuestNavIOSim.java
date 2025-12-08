package frc.robot.subsystems.questnav;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.SwerveDrive;
import gg.questnav.questnav.PoseFrame;
import java.util.Random;
import java.util.function.Supplier;

/**
 * Simulates the QuestNav IO by providing pose data from the swerve subsystem.
 */
public class QuestNavIOSim implements QuestNavIO {

    // Noise parameters
    private static final Distance TRANSLATION_NOISE_STDDEV_METERS = Centimeters.of(0.5);
    private static final Distance Z_NOISE_STDDEV_METERS = Centimeters.of(0.25);
    private static final Angle ROTATION_NOISE_STDDEV_RADIANS = Degrees.of(1.0);

    private final Random random = new Random();

    private Supplier<Pose2d> simulatedPoseSupplier;
    private int frameCounter = 1;

    public QuestNavIOSim(SwerveDrive swerveSubsystem) {
        simulatedPoseSupplier = swerveSubsystem::getActualPose;
    }

    /**
     * @return The simulated pose with added noise.
     */
    private Pose3d getSimulatedPose() {
        Pose2d pose2d = simulatedPoseSupplier.get();

        // Add some noise to the simulated pose
        double noiseX = random.nextGaussian() * TRANSLATION_NOISE_STDDEV_METERS.in(Meters);
        double noiseY = random.nextGaussian() * TRANSLATION_NOISE_STDDEV_METERS.in(Meters);
        double noiseZ = random.nextGaussian() * Z_NOISE_STDDEV_METERS.in(Meters);
        double noiseYaw = random.nextGaussian() * ROTATION_NOISE_STDDEV_RADIANS.in(Radians);

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
