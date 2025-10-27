package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.objectdetection.NeuralDetectorPhotonSim.NeuralDetectorSimPipeline;

/**
 * Extension of Vision subsystem with simulation support.
 */
public class VisionSim extends Vision {

    /**
     * A singleton instance of the vision system simulation.
     */
    private static VisionSystemSim photonVisionSimSystem;

    /**
     * @return The singleton vision system simulation.
     */
    public static VisionSystemSim getVisionSim() {
        // Initialize vision sim if needed
        if (photonVisionSimSystem == null) {
            photonVisionSimSystem = new VisionSystemSim("main");
            photonVisionSimSystem.addAprilTags(VisionConstants.aprilTagLayout);
        }

        return photonVisionSimSystem;
    }

    /**
     * The supplier for the robot pose to use in simulation.
     */
    public Supplier<Pose2d> robotPoseSupplier;

    /**
     * Creates a new VisionSim.
     * @param consumer - The vision consumer for pose measurements.
     * @param robotPoseSupplier - Supplier for the robot pose to use in simulation.
     * @param pipelines - The neural detector pipelines.
     * @param ios - The vision IO implementations.
     */
    public VisionSim(VisionConsumer consumer, Supplier<Pose2d> robotPoseSupplier, NeuralDetectorSimPipeline[] pipelines, VisionIO... ios) {
        super(consumer, ios);
        this.robotPoseSupplier = robotPoseSupplier;
    }


    @Override
    public void periodic() {
        // Update vision sim with current robot pose
        getVisionSim().update(robotPoseSupplier.get());

        super.periodic();
    }
}
