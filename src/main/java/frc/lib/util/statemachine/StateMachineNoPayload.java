package frc.lib.util.statemachine;

/**
 * A state machine that does not use payload data when changing states.
 * @param <TStateEnum> - The enum type representing all the possible states.
 */
public class StateMachineNoPayload<TStateEnum extends Enum<TStateEnum>> extends StateMachine<TStateEnum, Void> {
    public StateMachineNoPayload(String dashboardKey, Class<TStateEnum> stateEnumClass) {
        super(dashboardKey, stateEnumClass);
    }
}
