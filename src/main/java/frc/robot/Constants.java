package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Contains constants for the robot. Swerve constants are located in {@link SwerveConstants}.
 */
public final class Constants {

    /**
     * The current mode, see {@link Mode}
     */
    public static final Mode currentMode = Mode.SIM;

    /**
     * An enum of modes, for AdvantageKit.
     */
    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY,
    }

    /**
     * The deadband for the sticks. This is the range of values that will be considered 0.
     */
    public static final double stickDeadband = 0.1;

    /**
     * Constants for the pose estimator. Includes the state and vision standard deviations.
     */
    public static final class PoseEstimator {

        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        public static final Matrix<N3, N1> VisionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }

    /**
     * Constants for the arm. Includes CAN Ids.
     */
    public static final class Arm {
        /**
         * Deadband for the arm.
         */
        public static final double armDeadband = 0.1;

        /**
         * The percent power (from 0-1) to run the intake/outtake
         */
        public static final double armPercentOutput = 0.15;

        public static final int clawMotorId = 12;
    }
}
