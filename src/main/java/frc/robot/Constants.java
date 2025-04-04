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
     * Whether or not to silence the reportJoystickUnpluggedWarning warning.
     */
    // ! IMPORTANT: Remember to set this to false before competition!
    public static final boolean silenceJoystickUnpluggedWarning = true;

    /**
     * Whether or not to enable tuning mode.
     * Tuning mode enables the use of the dashboard to change values.
     */
    public static final boolean tuningMode = true;

    /**
     * Whether or not to disable HAL (Hardware Abstraction Layer) calls.
     * This is useful for testing without hardware.
     */
    public static final boolean disableHAL = false;

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
        public static final Matrix<N3, N1> VisionStdDevs = VecBuilder.fill(2, 2, 2);
    }
}
