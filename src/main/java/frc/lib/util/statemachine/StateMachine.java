package frc.lib.util.statemachine;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.statemachine.StateMachine.StateCycle.StateCycleBehavior;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * A generic finite state machine for managing the states of a subsystem.
 * A list of possible states is defined using an enum type.
 * Each enum type can have additional data and behavior defined in a {@link StateMachineState} object.
 * @param <TStateType> - The enum type representing all the possible states.
 */
public class StateMachine<TStateType extends Enum<TStateType>> {

    @FunctionalInterface
    public interface OnStateChangeAction<TEnumType extends Enum<TEnumType>> {
        /**
         * A functional interface representing an action to perform when a state change occurs.
         * @param previousState - The previous state before the change.
         */
        public void onStateChange(Optional<TEnumType> previousState);
    }

    /**
     * A cycle of states. Mainly used for controller inputs that toggle/cycle through a set of states.
     */
    public static class StateCycle<TEnumType extends Enum<TEnumType>> {

        /**
         * The behavior of the state cycle when cycling through states.
         */
        public enum StateCycleBehavior {
            /**
             * Rely on the current state of the state machine to determine the next state in the cycle.
             */
            RELY_ON_CURRENT_STATE,

            /**
             * Rely on the index of the state in the cycle to determine the next state.
             */
            RELY_ON_INDEX,
        }

        private final StateMachine<TEnumType> stateMachine;
        private final List<TEnumType> states;
        private final StateCycleBehavior behavior;

        private int currentIndex = 0;

        /**
         * Creates a new StateCycle with the specified states and behavior.
         * @param stateMachine - The state machine to cycle through.
         * @param states - The states to cycle through.
         * @param behavior - The behavior of the state cycle.
         */
        public StateCycle(StateMachine<TEnumType> stateMachine, List<TEnumType> states, StateCycleBehavior behavior) {
            this.stateMachine = stateMachine;
            this.states = states;
            this.behavior = behavior;
        }

        /**
         * Creates a new StateCycle with the specified states. Behavior defaults to {@link StateCycleBehavior#RELY_ON_CURRENT_STATE}.
         * @param stateMachine - The state machine to cycle through.
         * @param states - The states to cycle through.
         */
        public StateCycle(StateMachine<TEnumType> stateMachine, List<TEnumType> states) {
            this(stateMachine, states, StateCycleBehavior.RELY_ON_CURRENT_STATE);
        }

        /**
         * @return The next state in the cycle depending on the behavior.
         * If the behavior is {@link StateCycleBehavior#RELY_ON_CURRENT_STATE} and the current state is not in the cycle, the optional will be empty.
         * Note: If the state list has duplicates, the first occurrence will be used (for {@link StateCycleBehavior#RELY_ON_CURRENT_STATE}).
         */
        public Optional<TEnumType> getNextState() {
            if (behavior == StateCycleBehavior.RELY_ON_CURRENT_STATE) {
                TEnumType currentState = stateMachine.getCurrentState().enumType;
                int index = states.indexOf(currentState);

                // If the current state is not in the cycle, return empty
                if (index == -1) {
                    return Optional.empty();
                }

                currentIndex = index;
            }

            // Get the next state in the cycle
            currentIndex = (currentIndex + 1) % states.size();

            try {
                return Optional.of(states.get(currentIndex));
            } catch (IndexOutOfBoundsException e) {
                // Should never happen, but just in case
                return Optional.empty();
            }
        }

        /**
         * Schedules the next state in the cycle to be set in the state machine.
         * If the behavior is {@link StateCycleBehavior#RELY_ON_CURRENT_STATE} and the current state is not in the cycle, nothing will be scheduled.
         */
        public void scheduleNextState() {
            Optional<TEnumType> nextState = getNextState();

            nextState.ifPresent(stateMachine::scheduleStateChange);
        }

        /**
         * Resets the state cycle to the given index.
         * For {@link StateCycleBehavior#RELY_ON_CURRENT_STATE}, this will have no effect.
         */
        public void reset(int initialIndex) {
            currentIndex = initialIndex % states.size();
        }

        /**
         * Resets the state cycle to the beginning.
         * For {@link StateCycleBehavior#RELY_ON_CURRENT_STATE}, this will have no effect.
         */
        public void reset() {
            reset(0);
        }
    }

    /**
     * The prefix key for logging to the dashboard.
     */
    private static final String DEFAULT_DASHBOARD_KEY = "StateMachines/";

    /**
     * A trigger that is active when the driver station/robot is disabled.
     */
    private static final Trigger onDriverStationDisable = new Trigger(DriverStation::isDisabled);

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
    public final String dashboardKey;

    /**
     * The class of the enum type representing the states of the state machine.
     */
    private final Class<TStateType> stateEnumClass;

    /**
     * A map from enum states to their corresponding {@link StateMachineState} objects.
     */
    private final Map<TStateType, StateMachineState<TStateType>> stateMap;

    /**
     * A map of actions to perform on state changes.
     * When the state machine changes to a state, all actions in the list for that state are executed once.
     */
    private final Map<TStateType, List<OnStateChangeAction<TStateType>>> onStateChangeActions;

    /**
     * A map from enum states to their corresponding actions to perform each period while in that state.
     */
    // TODO: Consider making into List<Runnable>
    public final Map<TStateType, Runnable> stateActions;

    /**
     * The state that has been scheduled to change to on the next update cycle. Can be null.
     * See {@link #scheduleStateChange}.
     */
    private StateMachineState<TStateType> scheduledStateChange = null;

    /**
     * Whether the state machine should return to the default state when disabled.
     */
    private boolean shouldReturnToDefaultStateOnDisable = false;

    /**
     * Constructs a state machine with the specified initial state.
     * State is logged to the dashboard with the given key.
     * @param dashboardKey - The key to use for logging to the dashboard. Added as a prefix to {@link #DEFAULT_DASHBOARD_KEY}.
     * @param stateEnumClass - The class of the enum type representing the states of the state machine. Used to initialize the internal {@link EnumMap}s. Ex. MyStateEnum.class
     */
    public StateMachine(String dashboardKey, Class<TStateType> stateEnumClass) {
        this.dashboardKey = DEFAULT_DASHBOARD_KEY + dashboardKey;
        this.stateEnumClass = stateEnumClass;

        // Init maps
        stateMap = new EnumMap<>(this.stateEnumClass);
        onStateChangeActions = new EnumMap<>(this.stateEnumClass);
        stateActions = new EnumMap<>(this.stateEnumClass);

        // Initialize action lists for each state
        for (TStateType state : this.stateEnumClass.getEnumConstants()) {
            onStateChangeActions.put(state, new ArrayList<>());
        }

        recordCurrentState();
        recordScheduledStateChange();

        // Schedule state machine updates
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(this::commandSchedulerLoop);

        // Disable handling
        onDriverStationDisable.onTrue(
            Commands.runOnce(() -> {
                if (
                    shouldReturnToDefaultStateOnDisable &&
                    defaultState != null &&
                    (currentState == null || !currentState.equals(defaultState))
                ) {
                    System.out.println(
                        "[StateMachine \"" +
                        dashboardKey +
                        "\"] Returning to default state on disable: " +
                        defaultState.enumType
                    );

                    setStateAndUpdate(defaultState.enumType);

                    scheduledStateChange = null;
                    recordScheduledStateChange();
                }
            }).ignoringDisable(true)
        );
    }

    /**
     * A function that runs every command scheduler cycle to handle scheduled state changes.
     */
    private void commandSchedulerLoop() {
        // Check if the scheduled state can be changed to the new state based on the requirements
        if (scheduledStateChange == null) return;
        if (!scheduledStateChange.canChangeCondition.canChange(currentState.enumType)) return;

        setStateAndUpdate(scheduledStateChange.enumType);

        scheduledStateChange = null;
        recordScheduledStateChange();
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
     * @param previousState - The previous state before the change. Can be null if there was no previous state.
     */
    private void executeOnStateChangeActions(TStateType previousState) {
        if (currentState == null) return;

        List<OnStateChangeAction<TStateType>> actions = onStateChangeActions.get(currentState.enumType);

        for (OnStateChangeAction<TStateType> action : actions) {
            action.onStateChange(Optional.ofNullable(previousState));
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
        TStateType previousState = (currentState != null) ? currentState.enumType : null;

        currentState = getStateObject(newState);
        recordCurrentState();

        executeOnStateChangeActions(previousState);
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
     * Sets whether the state machine should return to the default state when disabled.
     * @param shouldReturn - Whether the state machine should return to the default state when disabled.
     */
    public void setReturnToDefaultStateOnDisable(boolean shouldReturn) {
        this.shouldReturnToDefaultStateOnDisable = shouldReturn;
    }

    /**
     * Sets whether the state machine should return to the default state when disabled, and returns the state machine for chaining.
     * @param shouldReturn - Whether the state machine should return to the default state when disabled.
     */
    public StateMachine<TStateType> withReturnToDefaultStateOnDisable(boolean shouldReturn) {
        setReturnToDefaultStateOnDisable(shouldReturn);
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
     * Adds an action to perform when the state machine changes to the specified state.
     * @param state - The state to add the action for.
     * @param action - The action to perform when the state machine changes to the specified state.
     * @throws IllegalArgumentException if the state does not exist in the state machine.
     */
    public void addOnStateChangeAction(TStateType state, Runnable action) {
        addOnStateChangeAction(state, previousState -> action.run());
    }

    /**
     * Adds an action to perform each period while the state machine is in the specified state.
     * @param state - The state to add the action for.
     * @param action - The action to perform each period while the state machine is in the specified state.
     */
    public void addStateAction(TStateType state, Runnable action) {
        stateActions.put(state, action);
    }

    /**
     * @param requirements - The subsystems required by the returned command.
     * @return A command that runs the state machine's current state's action each period.
     */
    public StateMachineSubsystemCommand<TStateType> getRunnableCommand(Subsystem... requirements) {
        return new StateMachineSubsystemCommand<>(this, requirements);
    }

    /**
     * Checks if the state machine is currently in the specified state.
     * @param state - The state to check.
     * @return True if the state machine is in the specified state, false otherwise.
     */
    public boolean is(TStateType state) {
        return getCurrentState().enumType == state;
    }

    /**
     * Returns a trigger that is active when the state machine is in the specified state.
     * See {@link #addOnStateChangeAction} for an better way to run actions on state changes.
     * @param state - The state to create a trigger for.
     */
    public Trigger on(TStateType state) {
        return new Trigger(() -> is(state));
    }

    /**
     * Creates a StateCycle with the specified states and behavior.
     * @param states - The states to cycle through.
     * @param behavior - The behavior of the state cycle.
     * @return The created StateCycle.
     */
    public StateCycle<TStateType> createStateCycle(List<TStateType> states, StateCycle.StateCycleBehavior behavior) {
        return new StateCycle<>(this, states, behavior);
    }

    /**
     * Creates a StateCycle with the specified states. Behavior defaults to {@link StateCycleBehavior#RELY_ON_CURRENT_STATE}.
     * @param states - The states to cycle through.
     * @return The created StateCycle.
     */
    public StateCycle<TStateType> createStateCycle(List<TStateType> states) {
        return new StateCycle<>(this, states);
    }
}
