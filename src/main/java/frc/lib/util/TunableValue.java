package frc.lib.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableValue implements DoubleSupplier {

    /**
     * A functional interface for a consumer that refreshes its config.
     * This is used to update the tunable numbers when the config is changed.
     */
    @FunctionalInterface
    public interface RefreshConfigConsumer {
        void accept();
    }

    /**
     * A list of all the consumers that need to be refreshed when the config is updated.
     * This is used to update the tunable numbers when the config is changed.
     */
    private static final List<RefreshConfigConsumer> refreshConfigConsumers = new ArrayList<>();

    /**
     * Adds a consumer to the list of consumers that need to be refreshed when the config is updated.
     * @param consumer The consumer to add
     */
    public static void addRefreshConfigConsumer(RefreshConfigConsumer consumer) {
        refreshConfigConsumers.add(consumer);
    }

    /**
     * Refreshes all the consumers in the list of consumers that need to be refreshed when the config is updated.
     */
    public static final Command refreshConfig = Commands.runOnce(() -> {
        refreshConfigConsumers.forEach(RefreshConfigConsumer::accept);
        refreshConfigConsumers.clear();
    });

    /**
     * The base key for the tuning table in NetworkTables.
     */
    private static final String BASE_TABLE_KEY = "/Tuning";

    /**
     * The key for the tunable number.
     */
    private final String key;

    /**
     * Whether or not the default value has been set.
     */
    private boolean hasDefault = false;

    /**
     * The default value of the number.
     */
    private double defaultValue;

    /**
     * The dashboard number that is used to get the value from the dashboard.
     */
    private LoggedNetworkNumber dashboardNumber;

    /**
     * A map of unique identifiers to the last value that was read for that identifier.
     * This is used to determine if the value has changed since the last time it was read.
     */
    private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    /**
     * A consumer to run when the value is refreshed.
     */
    // private DoubleConsumer onRefresh;

    /**
     * Create a new TunableValue
     * @param dashboardKey - Key on dashboard
     */
    public TunableValue(String dashboardKey) {
        this.key = BASE_TABLE_KEY + "/" + dashboardKey;
    }

    /**
     * Create a new TunableValue with the default value
     * @param dashboardKey - Key on dashboard
     * @param defaultValue - Default value
     */
    public TunableValue(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    /**
     * Create a new TunableValue with the default value
     * @param dashboardKey - Key on dashboard
     * @param defaultValue - Default value
     * @param onRefresh - The consumer to run when it is refreshed
     */
    public TunableValue(String dashboardKey, double defaultValue, DoubleConsumer onRefresh) {
        this(dashboardKey, defaultValue);
        // this.onRefresh = onRefresh;

        // TunableValue.addRefreshConfigConsumer(onRefresh);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     * @param defaultValue The default value
     */
    public void initDefault(double defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            if (Constants.tuningMode && !Constants.disableHAL) {
                dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     * @return The current value
     */
    public double get() {
        if (!hasDefault) {
            return 0.0;
        } else {
            return Constants.tuningMode && !Constants.disableHAL ? dashboardNumber.get() : defaultValue;
        }
    }

    /**
     * Checks whether the number has changed since our last check
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple objects. Recommended approach is to pass the result of "hashCode()"
     * @return True if the number has changed since the last time this method was called, false otherwise.
     */
    public boolean hasChanged(int id) {
        double currentValue = get();
        Double lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }

        return false;
    }

    /**
     * Runs action if any of the tunableNumbers have changed
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple objects. Recommended approach is to pass the result of "hashCode()"
     * @param action Callback to run when any of the tunable numbers have changed. Access tunable numbers in order inputted in method
     * @param tunableNumbers All tunable numbers to check
     */
    public static void ifChanged(int id, Consumer<double[]> action, TunableValue... tunableNumbers) {
        if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
            action.accept(Arrays.stream(tunableNumbers).mapToDouble(TunableValue::get).toArray());
        }
    }

    /** Runs action if any of the tunableNumbers have changed */
    public static void ifChanged(int id, Runnable action, TunableValue... tunableNumbers) {
        ifChanged(id, values -> action.run(), tunableNumbers);
    }

    @Override
    public double getAsDouble() {
        return get();
    }
}
