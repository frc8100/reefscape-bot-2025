package frc.lib.util.statemachine;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * A subsystem that can be in one of several states.
 * @param <TStateType> - The enum type representing the states of the subsystem.
 */
public class StateMachine<TStateType extends Enum<TStateType>> {

    /**
     * The prefix key for logging to the dashboard.
     */
    private static final String DEFAULT_DASHBOARD_KEY = "StateMachines";

    /**
     * The current state of the subsystem.
     */
    private StateMachineState<TStateType> currentState = null;

    /**
     * The default state of the subsystem.
     * Used to initialize the current state if it is null.
     * Also used to reset the state machine to its default state.
     */
    private StateMachineState<TStateType> defaultState = null;

    private final String dashboardKey;

    /**
     * A map of all possible states in the state machine.
     */
    private final Map<TStateType, StateMachineState<TStateType>> stateMap = new HashMap<>();

    /**
     * Constructs a state machine with the specified initial state.
     * State is logged to the dashboard with the given key.
     * @param dashboardKey - The key to use for logging to the dashboard. Added as a prefix to {@link #DEFAULT_DASHBOARD_KEY}.
     */
    public StateMachine(String dashboardKey) {
        // this.currentState = initialState;
        this.dashboardKey = dashboardKey + "/" + DEFAULT_DASHBOARD_KEY;
        // Record initial state to dashboard
        // Logger.recordOutput(dashboardKey, initialState);
    }

    /**
     * Sets the current state of the state machine.
     * @param newState - The new state to set.
     * @return True if the state was changed, false if it was the same as the current state.
     */
    public boolean setState(StateMachineState<TStateType> newState) {
        // Check if the new state is the same as the current state
        if (newState == getCurrentState()) {
            return false;
        }

        // Check if the new state exists in the state machine
        if (!stateMap.containsKey(newState.enumType)) {
            throw new IllegalArgumentException("StateMachine does not contain state: " + newState);
        }

        // Check if the state can be changed to the new state based on the requirements
        if (!newState.canChangeCondition.canChange(currentState.enumType)) {
            return false;
        }

        currentState = newState;
        Logger.recordOutput(dashboardKey, currentState);

        return true;
    }

    /**
     * Resets the state machine to its default state.
     * ! IMPORTANT: Bypasses any transition requirements.
     */
    public void reset() {
        if (defaultState == null) return;

        currentState = defaultState;
    }

    /**
     * Adds a state to the state machine.
     * @param state - The state to add.
     */
    public void addState(StateMachineState<TStateType> state) {
        stateMap.put(state.enumType, state);
    }

    /**
     * Adds a state to the state machine and returns the state machine for chaining.
     * @param state - The state to add.
     */
    public StateMachine<TStateType> withState(StateMachineState<TStateType> state) {
        addState(state);
        return this;
    }

    /**
     * Sets the default state of the state machine.
     * Also sets the current state to the default state if the current state is null.
     * @param defaultState - The default state to set.
     */
    public void setDefaultState(StateMachineState<TStateType> defaultState) {
        this.defaultState = defaultState;

        // If currentState is null, set it to the default state
        if (currentState == null) {
            currentState = defaultState;
            Logger.recordOutput(dashboardKey, currentState);
        }
    }

    /**
     * Sets the default state of the state machine and returns the state machine for chaining.
     * @param defaultState - The default state to set.
     */
    public StateMachine<TStateType> withDefaultState(StateMachineState<TStateType> defaultState) {
        setDefaultState(defaultState);
        return this;
    }

    /**
     * @return The current state of the state machine.
     * @throws IllegalStateException if the state machine does not have a current state defined and no default state to fall back on.
     */
    public StateMachineState<TStateType> getCurrentState() {
        if (currentState == null) {
            // Set to default state if it exists
            // This should never happen if the state machine is properly initialized
            if (defaultState != null) {
                currentState = defaultState;
                Logger.recordOutput(dashboardKey, currentState);
            } else {
                throw new IllegalStateException("StateMachine does not have a current state defined.");
            }
        }

        return currentState;
    }

    /**
     * Returns a trigger that is active when the state machine is in the specified state.
     * @param state - The state to create a trigger for.
     */
    public Trigger getStateTrigger(TStateType state) {
        return new Trigger(() -> getCurrentState().enumType == state);
    }
}
