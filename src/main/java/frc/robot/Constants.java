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
     * The current mode, see {@link Mode}.
     * Automatically set by the robot, unless set to Mode.REPLAY.
     */
    public static Mode currentMode = Mode.REAL;

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
     * Constants for the claw. Includes CAN Ids.
     */
    // TODO: Move to its own file
    public static final class Claw {
        /**
         * Deadband for the claw (controller input).
         */
        public static final double armDeadband = 0.1;

        /**
         * The percent power (from 0-1) to run the intake/outtake
         */
        public static final double armPercentOutput = 0.15;

        // Turn motor configs
        public static final int angleClawMotorId = 12;
        // TODO: Configure this
        public static final double angleGearRatio = 10.0;
        public static final boolean angleMotorInverted = false;
        public static final int angleMotorCurrentLimit = 40;

        // PID configs
        public static final double angleKP = 0.1;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /**
         * The maximum power for the angle motor, from 0-1.
         */
        public static final double anglePower = 0.8;

        // Outake motor configs
        public static final int outakeClawMotorId = 13;
        public static final double outakeGearRatio = 1.0;
        public static final boolean outakeMotorInverted = false;
        public static final int outakeMotorCurrentLimit = 40;

        /**
         * The maximum power for the outake motor, from 0-1.
         */
        public static final double outakePower = 0.7;
    }
}
