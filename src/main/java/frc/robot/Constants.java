package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {

    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY,
    }

    public static final double stickDeadband = 0.1;

    public static final class PoseEstimator {

        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        public static final Matrix<N3, N1> VisionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }

    public static final class Arm {

        public static final int kCanId = 11;
        public static final int kCurrentLimit = 40;
    }

    public static final class Intake {

        public static final int kCanId = 7;
        public static final boolean kMotorInverted = true;
        public static final int kCurrentLimit = 40;

        public static final double kIntakePower = 0.7;

        public static final double kRetractDistance = -3.5;

        public static final double kShotFeedTime = 1.0;
    }

    public static final class Launcher {

        public static final int kTopCanId = 12;
        public static final int kBottomCanId = 10;

        public static final int kCurrentLimit = 40;

        public static final double kTopPower = -1.0;
        public static final double kBottomPower = -1.0;

        public static final double kTopPowerIn = -0.7;
        public static final double kBottomPowerIn = -0.7;
    }
}
