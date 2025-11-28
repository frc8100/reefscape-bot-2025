package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.ArrayList;
import java.util.List;

/**
 * Contains all the CAN IDs and utilities to detect connection disruptions with alerts.
 */
public class CANIdConnections {

    private CANIdConnections() {}

    /**
     * The alert group for CAN ID alerts.
     */
    public static final String ALERT_GROUP = "CANAlerts";

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
     * Gets the CAN IDs for a module based on its index.
     * @param index - The index of the module (0-3). In FL, FR, BL, BR order.
     * @return The CAN IDs for the module.
     * @throws IllegalArgumentException - If the index is not between 0 and 3.
     */
    public static SwerveModuleCanIDs getModuleCANIdsFromIndex(int index) {
        return switch (index) {
            case 0 -> FRONT_LEFT_MODULE_CAN_IDS;
            case 1 -> FRONT_RIGHT_MODULE_CAN_IDS;
            case 2 -> BACK_LEFT_MODULE_CAN_IDS;
            case 3 -> BACK_RIGHT_MODULE_CAN_IDS;
            default -> throw new IllegalArgumentException("Invalid module index: " + index);
        };
    }

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
     * Alert for CAN bus disruptions (two or more consecutive disconnected CAN IDs).
     * Empty message; will be set in {@link #periodic()}
     */
    public static final Alert canBusDisruptionAlert = new Alert(ALERT_GROUP, "", AlertType.kError);

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

    /**
     * Periodically checks the connection status of all registered CAN ID alerts and updates the CAN bus disruption alert accordingly.
     */
    public static void periodic() {
        // TODO: maybe optimize by caching disconnected IDs instead of computing
        List<Integer> disconnectedIds = new ArrayList<>();

        for (CANIdAlert alert : canIdAlerts) {
            if (!alert.isConnected()) {
                // Add the CAN ID to the disconnected list
                disconnectedIds.add(alert.canId);
            }
        }

        List<Integer> disruptions = getDisruptions(disconnectedIds);

        // If no disruptions, clear alert and return
        if (disruptions.isEmpty()) {
            canBusDisruptionAlert.set(false);
            return;
        }

        StringBuilder disruptionMessage = new StringBuilder("CAN bus disruption detected at connections: ");

        // Set alert based on disruptions
        for (int i = 0; i < disruptions.size(); i++) {
            int disruptionIndex = disruptions.get(i);

            disruptionMessage.append(disruptionIndex);

            if (i < disruptions.size() - 1) {
                disruptionMessage.append(", ");
            }
        }

        // Finalize and set alert
        disruptionMessage.append(".");
        canBusDisruptionAlert.setText(disruptionMessage.toString());
        canBusDisruptionAlert.set(true);
    }
}
