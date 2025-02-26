package frc.robot.subsystems.swerve;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

/**
 * This file comes with command robot projects, and is intended to contain configuration
 * information. I think it would be prudent if this file only contained CanIDs, because it is useful
 * to have all the ids for the whole robot in one place. other configuration goes into subsystem
 * specific configuration files, to make sure this one isn't cluttered.
 */
public final class SwerveConstants {

    public static final double stickDeadband = 0.1;
    public static final double limelightOffset = 3;

    /**
     * Multiplier to the input chassis speeds of the swerve. Used temporarily to test.
     */
    public static final double debugSpeedMultiplier = 0.7;

    /**
     * @return The Pathplanner RobotConfig
     */
    // TODO: instead of loading from GUI, declare explicitly
    public static RobotConfig getRobotConfig() {
        // Load the RobotConfig from the GUI settings.
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            config = null;
        }

        return config;
    }

    /**
     * Whether to use open loop control.
     * Default is `true`
     */
    public static final boolean isOpenLoop = true;

    public static final class REV {

        public static final int pigeonID = 17;
    }

    /* Module Specific Constants */
    public static final class Swerve {

        /* Front Left Module */
        public static final class Mod0 {

            public static final int driveMotorID = 12;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset =
                    Rotation2d.fromDegrees(17.402344); // Rotation2d.fromDegrees(37.7);
            public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module */
        public static final class Mod1 {

            public static final int driveMotorID = 5;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees((79.277344) + 180.0);
            public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module */
        public static final class Mod2 {

            public static final int driveMotorID = 4;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 15;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees((73.740234));
            public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module */
        public static final class Mod3 {

            public static final int driveMotorID = 1;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 16;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees((185.273438) + 180.0);
            public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
}
