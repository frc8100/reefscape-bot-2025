package frc.lib.util.statemachine;

/**
 * A possible state that a {@link StateMachine} can be in.
 * @param <TEnumType> - The enum type representing the states of the state machine.
 */
public class StateMachineState<TEnumType extends Enum<TEnumType>> {

    @FunctionalInterface
    public interface StateChangeCondition<TEnumType extends Enum<TEnumType>> {
        /**
         * A functional interface representing a condition to determine if a state change can occur.
         * @param previousState - The previous state before the change.
         */
        public boolean canChange(TEnumType previousState);
    }

    /**
     * The enum type representing the state. This is used for equality checks.
     */
    public final TEnumType enumType;

    /**
     * The name of the state, used for logging and debugging.
     */
    public final String name;

    /**
     * A supplier that determines if the state can be changed to this state.
     * By default, it always returns true.
     */
    public final StateChangeCondition<TEnumType> canChangeCondition;

    /**
     * Constructs a StateMachineState with the specified enum type, name, and change condition.
     * @param enumType - The enum type representing the state.
     * @param name - The name of the state.
     * @param canChangeCondition - A condition that determines if the state can be changed to this state.
     */
    public StateMachineState(TEnumType enumType, String name, StateChangeCondition<TEnumType> canChangeCondition) {
        this.enumType = enumType;
        this.name = name;
        this.canChangeCondition = canChangeCondition;
    }

    /**
     * Constructs a StateMachineState with the specified enum type and name.
     */
    public StateMachineState(TEnumType enumType, String name) {
        this(enumType, name, previousState -> true);
    }

    @Override
    public String toString() {
        return name;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        StateMachineState<?> other = (StateMachineState<?>) obj;
        return enumType.equals(other.enumType);
    }

    @Override
    public int hashCode() {
        return enumType.hashCode();
    }
}
