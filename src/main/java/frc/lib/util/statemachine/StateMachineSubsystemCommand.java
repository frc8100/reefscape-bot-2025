package frc.lib.util.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that executes actions based on the current state of a StateMachine.
 */
public class StateMachineSubsystemCommand<TStateType extends Enum<TStateType>> extends Command {

    private final StateMachine<TStateType> stateMachine;

    public StateMachineSubsystemCommand(StateMachine<TStateType> stateMachine, Subsystem... requirements) {
        this.stateMachine = stateMachine;
        addRequirements(requirements);
    }

    @Override
    public void execute() {
        TStateType currentState = stateMachine.getCurrentState().enumType;
        Runnable action = stateMachine.stateActions.get(currentState);

        if (action != null) {
            action.run();
        }
    }
}
