package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

/**
 * Contains all the CAN IDs and utilities to detect connection disruptions with alerts.
 */
public class CANIdConnections {

    private CANIdConnections() {}

    /**
     * List of CAN ID alerts for monitoring CAN device connections.
     */
    private static final List<CANIdAlert> canIdAlerts = new ArrayList<>();

    /**
     * Registers a CAN ID alert to be monitored.
     * @param alert - The CAN ID alert to register.
     */
    public static void registerCANIdAlert(CANIdAlert alert) {
        canIdAlerts.add(alert);
    }

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

    /**
     * List of all CAN IDs in the order the CAN bus is wired, starting from the PDP and ending at the RoboRIO.
     * This is used to detect connection disruptions.
     */
    // TODO: set this
    public static final List<Integer> canIdConnectionsInOrder = List.of(
        PIGEON_ID,
        FRONT_LEFT_MODULE_CAN_IDS.driveMotorID,
        FRONT_LEFT_MODULE_CAN_IDS.angleMotorID,
        FRONT_LEFT_MODULE_CAN_IDS.canCoderID,
        FRONT_RIGHT_MODULE_CAN_IDS.driveMotorID,
        FRONT_RIGHT_MODULE_CAN_IDS.angleMotorID,
        FRONT_RIGHT_MODULE_CAN_IDS.canCoderID,
        BACK_LEFT_MODULE_CAN_IDS.driveMotorID,
        BACK_LEFT_MODULE_CAN_IDS.angleMotorID,
        BACK_LEFT_MODULE_CAN_IDS.canCoderID,
        BACK_RIGHT_MODULE_CAN_IDS.driveMotorID,
        BACK_RIGHT_MODULE_CAN_IDS.angleMotorID,
        BACK_RIGHT_MODULE_CAN_IDS.canCoderID
    );

    /**
     * Gets a list connections that are disrupted based on the list of disconnected CAN IDs.
     * A disruption is when two or more consecutive CAN IDs are disconnected.
     * If three or more consecutive CAN IDs are disconnected, that counts as one disruption starting at the first disconnected CAN ID.
     * A disruption at index `i` means that the connection between `canIdConnectionsInOrder[i - 1]` and `canIdConnectionsInOrder[i]` is disrupted.
     * (Connection 0 is between the PDP and the first CAN ID in the list.)
     * @param disconnectedIds - The list of disconnected CAN IDs.
     * @return A list of indices where disruptions occur.
     */
    public static List<Integer> getDisruptions(List<Integer> disconnectedIds) {
        List<Integer> disruptions = new ArrayList<>();

        /**
         * The number of consecutive disconnected CAN IDs seen so far.
         */
        int consecutive = 0;

        for (int i = 0; i < canIdConnectionsInOrder.size(); i++) {
            int canId = canIdConnectionsInOrder.get(i);

            if (disconnectedIds.contains(canId)) {
                consecutive++;

                // Start of a new disruption
                if (consecutive == 2) {
                    disruptions.add(i);
                }
            } else {
                // Reset consecutive count
                consecutive = 0;
            }
        }

        return disruptions;
    }
}
