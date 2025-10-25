package frc.robot;

/**
 * Contains constants for the robot. Subsystem-specific constants should go in their own files.
 */
public final class Constants {

    private Constants() {}

    /**
     * The period of the main robot loop, in seconds.
     */
    public static final double LOOP_PERIOD_SECONDS = 0.02;

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
    public static final boolean enableSysId = false;

    /**
     * Whether or not to store WPILog files during simulation.
     */
    public static final boolean storeSimLogs = false;

    /**
     * The directory to store WPILog files during simulation.
     */
    public static final String simLogDirectory = "C:\\Users\\Public\\wpilib\\2025\\logs";

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
}
