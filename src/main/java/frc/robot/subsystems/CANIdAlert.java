package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Contains an alert for a specific CAN ID to monitor its connection status.
 */
public class CANIdAlert {

    /**
     * The default debounce time for CAN connection alerts.
     */
    public static final Time DEFAULT_DEBOUNCE_TIME = Seconds.of(0.5);

    /**
     * A debouncer to prevent alert flapping.
     */
    private final Debouncer connectionDebouncer = new Debouncer(DEFAULT_DEBOUNCE_TIME.in(Seconds));

    /**
     * The alert to be triggered on disconnection.
     */
    private final Alert disconnectionAlert;

    private boolean isConnected = true;

    /**
     * Creates a CAN ID alert for the given CAN ID and device name.
     * @param canId - The CAN ID of the device.
     * @param deviceName - The name of the device.
     */
    public CANIdAlert(int canId, String deviceName) {
        this.disconnectionAlert = new Alert(
            "Disconnected CAN device: " + deviceName + " (ID " + Integer.toString(canId) + ").",
            AlertType.kError
        );
    }

    /**
     * Updates the connection status of the CAN device. Sets the alert if disconnected.
     * @param currentlyConnected - Whether the device is currently connected.
     */
    public void updateConnectionStatus(boolean currentlyConnected) {
        this.isConnected = connectionDebouncer.calculate(currentlyConnected);

        disconnectionAlert.set(!isConnected);
    }
}
