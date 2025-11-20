package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CANIdConnections;

/**
 * Contains module-specific constants for the swerve drive.
 * For CAN IDs, see {@link CANIdConnections}.
 * For general swerve constants, see {@link SwerveConfig}.
 */
public final class SwerveModuleSpecificConstants {

    private SwerveModuleSpecificConstants() {}

    /**
     * Swerve module constants to be used when creating swerve modules.
     * @param angleOffset - The angle offset of the module.
     */
    public static record RobotSwerveModuleConstants(Rotation2d angleOffset) {}

    public static final RobotSwerveModuleConstants FRONT_LEFT_MODULE_CONSTANTS = new RobotSwerveModuleConstants(
        new Rotation2d(2.922 + Math.PI - 0.626)
    );

    public static final RobotSwerveModuleConstants FRONT_RIGHT_MODULE_CONSTANTS = new RobotSwerveModuleConstants(
        new Rotation2d(2.635 - 0.222 - 0.845)
    );

    public static final RobotSwerveModuleConstants BACK_LEFT_MODULE_CONSTANTS = new RobotSwerveModuleConstants(
        new Rotation2d(1.267 + Math.PI - 0.004)
    );

    public static final RobotSwerveModuleConstants BACK_RIGHT_MODULE_CONSTANTS = new RobotSwerveModuleConstants(
        new Rotation2d(-3.053 - 0.021)
    );
}
