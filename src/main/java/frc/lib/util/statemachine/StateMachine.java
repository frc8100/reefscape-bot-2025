package frc.lib.util.statemachine;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * A subsystem that can be in one of several states.
 * @param <TStateType> - The enum type representing the states of the subsystem.
 */
public class StateMachine<TStateType extends Enum<TStateType>> {

    @FunctionalInterface
    public interface OnStateChangeAction<TEnumType extends Enum<TEnumType>> {
        /**
         * A functional interface representing an action to perform when a state change occurs.
         * @param previousState - The previous state before the change.
         */
        public void onStateChange(TEnumType previousState);
    }

    /**
     * The prefix key for logging to the dashboard.
     */
    private static final String DEFAULT_DASHBOARD_KEY = "StateMachines/";

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

    /**
     * The key used for logging to the dashboard.
     * Example: "StateMachines/Swerve"
     */
    private final String dashboardKey;

    /**
     * A map of all possible states in the state machine.
     */
    private final Map<TStateType, StateMachineState<TStateType>> stateMap = new HashMap<>();

    /**
     * A map of actions to perform on state changes.
     */
    private final Map<TStateType, List<OnStateChangeAction<TStateType>>> onStateChangeActions = new HashMap<>();

    /**
     * The state that has been scheduled to change to on the next update cycle.
     * See {@link #scheduleStateChange}.
     */
    private StateMachineState<TStateType> scheduledStateChange = null;

    /**
     * Constructs a state machine with the specified initial state.
     * State is logged to the dashboard with the given key.
     * @param dashboardKey - The key to use for logging to the dashboard. Added as a prefix to {@link #DEFAULT_DASHBOARD_KEY}.
     */
    public StateMachine(String dashboardKey) {
        this.dashboardKey = DEFAULT_DASHBOARD_KEY + dashboardKey;

        recordCurrentState();
        recordScheduledStateChange();

        // Schedule state machine updates
        CommandScheduler.getInstance()
            .getDefaultButtonLoop()
            .bind(() -> {
                if (scheduledStateChange == null) return;

                // Check if the state can be changed to the new state based on the requirements
                if (!scheduledStateChange.canChangeCondition.canChange(currentState.enumType)) return;

                setStateAndUpdate(scheduledStateChange.enumType);

                scheduledStateChange = null;
                recordScheduledStateChange();
            });
    }

    /**
     * @param state - The enum state to get the corresponding StateMachineState object for.
     * @return The StateMachineState object corresponding to the given enum state.
     * @throws IllegalArgumentException if the state does not exist in the state machine.
     */
    private StateMachineState<TStateType> getStateObject(TStateType state) {
        if (!stateMap.containsKey(state)) {
            throw new IllegalArgumentException("StateMachine does not contain state: " + state);
        }

        return stateMap.get(state);
    }

    /**
     * Executes all actions registered for the current state change.
     * @param previousState - The previous state before the change.
     */
    private void executeOnStateChangeActions(TStateType previousState) {
        if (currentState == null) return;

        List<OnStateChangeAction<TStateType>> actions = onStateChangeActions.get(currentState.enumType);

        for (OnStateChangeAction<TStateType> action : actions) {
            action.onStateChange(previousState);
        }
    }

    /**
     * Records the current state to the dashboard.
     */
    private void recordCurrentState() {
        if (currentState == null) {
            Logger.recordOutput(dashboardKey + "/State", "None");
            return;
        }

        Logger.recordOutput(dashboardKey + "/State", currentState.toString());
    }

    /**
     * Records the scheduled state change to the dashboard.
     */
    private void recordScheduledStateChange() {
        if (scheduledStateChange == null) {
            Logger.recordOutput(dashboardKey + "/ScheduledStateChange", "None");
            return;
        }

        Logger.recordOutput(dashboardKey + "/ScheduledStateChange", scheduledStateChange.toString());
    }

    /**
     * Sets the current state of the state machine, runs the onStateChange actions, and records the state change.
     * Bypasses any transition requirements.
     * @param newState - The new state to set, as the enum type.
     */
    private void setStateAndUpdate(TStateType newState) {
        if (currentState != null) {
            executeOnStateChangeActions(currentState.enumType);
        }

        currentState = getStateObject(newState);
        recordCurrentState();
    }

    /**
     * Sets the current state of the state machine.
     * @param newState - The new state to set, as the enum type.
     * @return True if the state was changed, false if it was the same as the current state.
     * @throws IllegalArgumentException if the new state does not exist in the state machine.
     */
    public boolean setState(TStateType newState) {
        // Check if the new state is the same as the current state
        if (newState == getCurrentState().enumType) {
            return false;
        }

        StateMachineState<TStateType> newStateObj = getStateObject(newState);

        // Check if the state can be changed to the new state based on the requirements
        if (!newStateObj.canChangeCondition.canChange(currentState.enumType)) {
            return false;
        }

        setStateAndUpdate(newState);

        return true;
    }

    /**
     * Forces the current state of the state machine to the specified state.
     * ! IMPORTANT: Bypasses any transition requirements.
     * @param newState - The new state to set, as the enum type.
     * @throws IllegalArgumentException if the new state does not exist in the state machine.
     */
    public void forceSetState(TStateType newState) {
        StateMachineState<TStateType> newStateObj = getStateObject(newState);

        setStateAndUpdate(newState);
    }

    /**
     * Schedules a state change to the specified state on the next update cycle.
     * If the state can be changed immediately, it will be changed immediately.
     * @param newState - The new state to change to, as the enum type.
     * @throws IllegalArgumentException if the new state does not exist in the state machine.
     */
    public void scheduleStateChange(TStateType newState) {
        StateMachineState<TStateType> newStateObj = getStateObject(newState);

        // If it can change now, change immediately
        if (newStateObj.canChangeCondition.canChange(currentState.enumType)) {
            setStateAndUpdate(newState);
            return;
        }

        scheduledStateChange = newStateObj;
        recordScheduledStateChange();
    }

    /**
     * Unschedule any scheduled state change.
     */
    public void unscheduleStateChange() {
        scheduledStateChange = null;
        recordScheduledStateChange();
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
        onStateChangeActions.put(state.enumType, new ArrayList<>());
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
            setStateAndUpdate(defaultState.enumType);
        }
    }

    /**
     * Sets and adds the default state of the state machine and returns the state machine for chaining.
     * @param defaultState - The default state to set.
     */
    public StateMachine<TStateType> withDefaultState(StateMachineState<TStateType> defaultState) {
        addState(defaultState);
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
                setStateAndUpdate(defaultState.enumType);
            } else {
                throw new IllegalStateException("StateMachine does not have a current state defined.");
            }
        }

        return currentState;
    }

    /**
     * Adds an action to perform when the state machine changes to the specified state.
     * @param state - The state to add the action for.
     * @param action - The action to perform when the state machine changes to the specified state.
     * @throws IllegalArgumentException if the state does not exist in the state machine.
     */
    public void addOnStateChangeAction(TStateType state, OnStateChangeAction<TStateType> action) {
        if (!onStateChangeActions.containsKey(state)) {
            throw new IllegalArgumentException("StateMachine does not contain state: " + state);
        }

        onStateChangeActions.get(state).add(action);
    }

    /**
     * Returns a trigger that is active when the state machine is in the specified state.
     * @param state - The state to create a trigger for.
     */
    public Trigger getStateTrigger(TStateType state) {
        return new Trigger(() -> getCurrentState().enumType == state);
    }
}
