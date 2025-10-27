package frc.robot.subsystems.vision.objectdetection;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.GamePieceObservation;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import limelight.networktables.target.pipeline.NeuralDetector;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

/**
 * Simulates the data for LimeLight Neural Detector in PhotonVision simulation.
 */
public class NeuralDetectorPhotonSim {

    /**
     * A pipeline configuration for a single object that can be detected by the Neural Detector.
     */
    public abstract static class NeuralDetectorSimPipeline {

        public String className;
        public double classID;
        public TargetModel targetModel;

        public Supplier<Pose3d[]> potentialTargetsSupplier;
    }

    /**
     * The target model for coral.
     */
    public static final TargetModel coralModel = new TargetModel(
        Inches.of(11.875).in(Meters),
        Inches.of(4).in(Meters),
        Inches.of(4).in(Meters)
    );

    /**
     * The target model for algae.
     */
    public static final TargetModel algaeModel = new TargetModel(Inches.of(16.25).in(Meters));

    private final VisionSystemSim systemSim;
    private final PhotonCameraSim cameraSim;
    private final NeuralDetectorSimPipeline[] pipelines;

    // private final Supplier<Pose2d> robotPoseSupplier;

    public NeuralDetectorPhotonSim(
        VisionSystemSim systemSim,
        PhotonCameraSim cameraSim,
        NeuralDetectorSimPipeline... pipelines
    ) {
        this.systemSim = systemSim;
        this.cameraSim = cameraSim;
        this.pipelines = pipelines;
        // this.robotPoseSupplier = robotPoseSupplier;

        // TODO: investigate adding/removing vision targets instead of simulating
        // systemSim.addVisionTargets(null, null);
        // systemSim.removeVisionTargets(null);
    }

    /**
     * Gets all unread game piece observations since the last call to this method. Should only be called once per update.
     * @return An array of all unread game piece observations since the last call to this method.
     */
    public List<GamePieceObservation> getUnreadObservations() {
        // cameraSim.prop.estMsUntilNextFrame();

        Optional<Pose3d> measuredCameraPose = systemSim.getCameraPose(cameraSim);

        // If the camera pose is not known, return no observations
        if (measuredCameraPose.isEmpty()) {
            return List.of();
        }

        Pose3d cameraPose = measuredCameraPose.get();

        List<GamePieceObservation> observations = new ArrayList<>();

        // Simulate object detection for each pipeline
        for (NeuralDetectorSimPipeline pipeline : pipelines) {
            Pose3d[] potentialTargets = pipeline.potentialTargetsSupplier.get();

            for (Pose3d targetPose : potentialTargets) {
                if (cameraSim.canSeeTargetPose(cameraPose, new VisionTargetSim(targetPose, pipeline.targetModel))) {
                    // TODO: Add detected target to the pipeline result
                    // return new GamePieceObservation[] {
                    //     // new GamePieceObservation(pipeline.className, pipeline.classID, targetPose),
                    // };
                    // observations.add(new GamePieceObservation() {});
                }
            }
        }

        return observations;
    }
}
