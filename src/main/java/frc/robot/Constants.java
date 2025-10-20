package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.swerve.SwerveConstants;

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
     * Whether or not to enable system identification mode.
     * - When true, SysId routines are available for use (in the AutoChooser).
     * - When true and in simulation, an empty SimulationArena will be used.
     */
    public static final boolean enableSysId = true;

    /**
     * Whether or not to store WPILog files during simulation.
     */
    public static final boolean storeSimLogs = false;

    /**
     * The directory to store WPILog files during simulation.
     */
    public static final String simLogDirectory = "D:\\wpilib\\2025\\logs";

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
     * @deprecated Use {@link #currentMode} instead.
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
