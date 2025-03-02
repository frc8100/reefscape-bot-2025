package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Contains constants for the robot. Swerve constants are located in {@link SwerveConstants}.
 */
public final class Constants {

    private Constants() {}

    /**
     * The current mode, see {@link Mode}.
     * Automatically set by the robot, unless set to Mode.REPLAY.
     */
    public static Mode currentMode = Mode.REAL;

    /**
     * An enum of modes for the robot.
     */
    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY,
    }

    /**
     * Constants for the pose estimator. Includes the state and vision standard deviations.
     */
    public static final class PoseEstimator {

        private PoseEstimator() {}

        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        public static final Matrix<N3, N1> VisionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }
}
