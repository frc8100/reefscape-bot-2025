package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Current;

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

    /**
     * Constants for the claw. Includes CAN Ids.
     */
    // TODO: Move to its own file
    public static final class Claw {
        private Claw() {}

        /**
         * Deadband for the claw (controller input).
         */
        public static final double ARM_CONTROLLER_DEADBAND = 0.1;

        /**
         * The percent power (from 0-1) to run the intake/outtake
         */
        public static final double ARM_MAX_OUTPUT = 0.15;

        // Turn motor configs
        public static final int ANGLE_MOTOR_ID = 12;
        public static final double ANGLE_GEAR_RATIO = 15.0;
        public static final boolean IS_ANGLE_MOTOR_INVERTED = false;
        public static final Current ANGLE_MOTOR_CURRENT_LIMIT = Amps.of(40);

        // PID configs
        public static final double ANGLE_KP = 0.1;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.0;
        public static final double ANGLE_KF = 0.0;

        /**
         * The maximum power for the angle motor, from 0-1.
         */
        public static final double MAX_ANGLE_POWER = 0.8;

        // Outtake motor configs
        public static final int OUTTAKE_MOTOR_ID = 13;
        public static final double OUTTAKE_GEAR_RATIO = 1.0;
        public static final boolean IS_OUTTAKE_MOTOR_INVERTED = false;
        public static final Current OUTTAKE_MOTOR_CURRENT_LIMIT = Amps.of(40);

        /**
         * The maximum power for the outtake motor, from 0-1.
         */
        public static final double MAX_OUTTAKE_POWER = 0.7;
    }
}
