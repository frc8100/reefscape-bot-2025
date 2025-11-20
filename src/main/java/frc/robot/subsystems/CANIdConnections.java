package frc.robot.subsystems;

/**
 * Contains all the CAN IDs and utilities to detect connection disruptions with alerts.
 */
public class CANIdConnections {

    private CANIdConnections() {}

    public static final int PIGEON_ID = 17;

    /**
     * Swerve Module CAN IDs to be used when creating swerve modules.
     * @param driveMotorID - The CAN ID of the drive motor.
     * @param angleMotorID - The CAN ID of the angle motor.
     * @param canCoderID - The CAN ID of the CANCoder.
     */
    public static record SwerveModuleCanIDs(int driveMotorID, int angleMotorID, int canCoderID) {}

    // Swerve Module CAN IDs
    public static final SwerveModuleCanIDs FRONT_LEFT_MODULE_CAN_IDS = new SwerveModuleCanIDs(12, 3, 14);
    public static final SwerveModuleCanIDs FRONT_RIGHT_MODULE_CAN_IDS = new SwerveModuleCanIDs(5, 2, 13);
    public static final SwerveModuleCanIDs BACK_LEFT_MODULE_CAN_IDS = new SwerveModuleCanIDs(4, 10, 15);
    public static final SwerveModuleCanIDs BACK_RIGHT_MODULE_CAN_IDS = new SwerveModuleCanIDs(1, 8, 16);
}
