package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

/**
 * Contains constants related to the Swerve Drive.
 * Mainly includes CAN IDs and angle offsets.
 */
public final class SwerveConstants {
    private SwerveConstants() {}

    public static final int PIGEON_ID = 17;

    /** Module Specific Constants */
    public static final class Swerve {
        private Swerve() {}

        /** Front Left Module */
        public static final class Mod0 {
            private Mod0() {}

            public static final int DRIVE_MOTOR_ID = 12;
            public static final int ANGLE_MOTOR_ID = 3;
            public static final int CANCODER_ID = 14;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(17.402344);
            public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /** Front Right Module */
        public static final class Mod1 {
            private Mod1() {}

            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final int CANCODER_ID = 13;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees((79.277344) + 180.0);
            public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /** Back Left Module */
        public static final class Mod2 {
            private Mod2() {}

            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 10;
            public static final int CANCODER_ID = 15;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees((73.740234));
            public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /** Back Right Module */
        public static final class Mod3 {
            private Mod3() {}

            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CANCODER_ID = 16;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees((185.273438) + 180.0);
            public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
    }
}
