package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.path.AutoRoutines;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ButtonBindings {

    // private enum ButtonBindingType {
    //     /**
    //      * A button that can be pressed or released.
    //      * @see XboxController.Button
    //      */
    //     BUTTON,

    //     /**
    //      * An axis that can be a set to a value between -1 and 1.
    //      * @see XboxController.Axis
    //      */
    //     AXIS,
    // }

    /**
     * A wrapper around GenericHID for creating button bindings.
     * Use {@link #createBinding} to create button bindings.
     *
     * Example usage:
     * {@code
     * Controller controller1 = new Controller(0);
     * controller1.createBinding(XboxController.Button.A, (trigger) -> trigger.onTrue(new Command()));
     */
    public static class Controller extends GenericHID {

        /**
         * The directions of a POV button.
         * @see POVButton
         */
        public enum POVButtonDirection {
            UP(0),
            RIGHT(90),
            DOWN(180),
            LEFT(270);

            public final int angle;

            POVButtonDirection(int angle) {
                this.angle = angle;
            }
        }

        // public final GenericHID controller;

        /**
         * Creates a controller on the specified port.
         * @param port - The port the controller is connected to as listed in the Driver Station.
         */
        public Controller(int port) {
            super(port);
        }

        /**
         * Creates a button binding on this controller.
         * See {@link ButtonBindingDeclaration} for usage.
         */
        // public ButtonBindingDeclaration createBinding(
        //     XboxController.Button buttonValue,
        //     Consumer<Trigger> buttonTriggerConsumer
        // ) {
        //     return new ButtonBindingDeclaration(this, buttonValue, buttonTriggerConsumer);
        // }
        // public void createBinding(int buttonValue, Consumer<Trigger> buttonTriggerConsumer) {
        //     buttonTriggerConsumer.accept(new JoystickButton(this, buttonValue));
        // }

        // public void createBinding(XboxController.Button buttonValue, Consumer<Trigger> buttonTriggerConsumer) {
        //     createBinding(buttonValue.value, buttonTriggerConsumer);
        // }

        public Trigger getJoystickButton(int buttonValue) {
            return new JoystickButton(this, buttonValue);
        }

        public Trigger getJoystickButton(XboxController.Button button) {
            return getJoystickButton(button.value);
        }

        public Trigger getPOVButton(int angle) {
            return new POVButton(this, angle);
        }

        public Trigger getPOVButton(POVButtonDirection direction) {
            return getPOVButton(direction.angle);
        }

        /**
         * Creates a button binding on this controller without a consumer.
         * See {@link ButtonBindingDeclaration} for usage.
         */
        // public ButtonBindingDeclaration createBinding(XboxController.Button buttonValue) {
        //     return new ButtonBindingDeclaration(this, buttonValue);
        // }

        /**
         * @param button - The button to get the state of.
         * @return A BooleanSupplier that returns true when the button is pressed.
         */
        public BooleanSupplier getButtonSupplier(XboxController.Button button) {
            return () -> this.getRawButton(button.value);
        }

        /**
         * @param axis - The axis to get the value of.
         * @return A DoubleSupplier that returns the value of the axis between -1 and 1.
         */
        public DoubleSupplier getAxisSupplier(XboxController.Axis axis) {
            return () -> this.getRawAxis(axis.value);
        }

        /**
         * @param axis - The axis to get the value of.
         * @param invert - Whether to invert the value of the axis.
         * @return A DoubleConsumer that sets the value of the axis between -1 and 1.
         */
        public DoubleSupplier getAxisSupplier(XboxController.Axis axis, boolean invert) {
            return () -> (invert ? -1 : 1) * this.getRawAxis(axis.value);
        }
    }

    /**
     * A wrapper for declaring button bindings in an enum.
     * Ex.
     * {@code new ButtonBindingDeclaration(joystick, XboxController.Button.A, (trigger) -> trigger.onTrue(new Command()))}
     * Equivalent to:
     * {@code new JoystickButton(joystick, XboxController.Button.A.value).onTrue(new Command())}
     */
    // this is more verbose than just using JoystickButton directly
    // public static class ButtonBindingDeclaration {

    //     /**
    //      * The ID of the button on the joystick.
    //      * @see XboxController.Button
    //      */
    //     private final int id;

    //     /**
    //      * A consumer that accepts a Trigger to bind commands to.
    //      * Ex. {@code (Trigger) -> trigger.onTrue(new Command())}
    //      */
    //     private final Consumer<Trigger> buttonTriggerConsumer;

    //     /**
    //      * The controller to query for the button state.
    //      */
    //     private final GenericHID joystick;

    //     /**
    //      * The Trigger that represents the button state.
    //      * Note that {@link JoystickButton} extends {@link Trigger}.
    //      * @see JoystickButton
    //      */
    //     private final Trigger trigger;

    //     /**
    //      * @return A BooleanSupplier that returns true when the button is pressed.
    //      */
    //     public BooleanSupplier getAsBooleanSupplier() {
    //         return () -> joystick.getRawButton(id);
    //     }

    //     /**
    //      * Creates a button binding declaration. Invokes the provided consumer with a Trigger.
    //      */
    //     public ButtonBindingDeclaration(
    //         GenericHID joystick,
    //         XboxController.Button buttonValue,
    //         Consumer<Trigger> buttonTriggerConsumer
    //     ) {
    //         this.joystick = joystick;
    //         this.id = buttonValue.value;
    //         this.buttonTriggerConsumer = buttonTriggerConsumer;
    //         this.trigger = new JoystickButton(joystick, id);

    //         // Immediately accept the trigger to bind commands to it
    //         buttonTriggerConsumer.accept(trigger);
    //     }

    //     /**
    //      * Creates a button binding declaration without a consumer.
    //      */
    //     public ButtonBindingDeclaration(GenericHID joystick, XboxController.Button buttonValue) {
    //         this(joystick, buttonValue, trigger -> {});
    //     }
    // }

    // Subsystem references
    private final AutoRoutines autoRoutines;
    private final SwerveDrive swerveSubsystem;
    private final Elevator elevatorSubsystem;
    private final Claw clawSubsystem;
    // private final Vision visionSubsystem;

    private final Controller driverController = new Controller(0);
    private final Controller operatorController = new Controller(1);

    public ButtonBindings(AutoRoutines autoRoutines) {
        this.autoRoutines = autoRoutines;

        // Get subsystems from AutoRoutines
        this.swerveSubsystem = autoRoutines.swerveSubsystem;
        this.elevatorSubsystem = autoRoutines.elevatorSubsystem;
        this.clawSubsystem = autoRoutines.clawSubsystem;
        // this.visionSubsystem = autoRoutines.visionSubsystem;
    }

    public void configureButtonBindings() {
        // Driver controller bindings

        // Controls.mainDriveControls
        //     .getJoystickButtonOf(Controls.mainDriveControls.zeroGyroButton)
        //     .onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
        // driverController.createBinding(Controls.mainDriveControls.zeroGyroButton, (trigger) ->
        //     trigger.onTrue(new InstantCommand(swerveSubsystem::zeroGyro))
        // );

        driverController
            .getJoystickButton(Controls.mainDriveControls.zeroGyroButton)
            .onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        // TODO: Align to reefs
        // Controls.mainDriveControls
        //     .getJoystickButtonOf(Controls.mainDriveControls.alignToLeftReefUsingVision)
        //     // .whileTrue(new AlignToReefTagRelative(false, swerveSubsystem));
        //     .whileTrue(new PhotonVisionAlign(false, swerveSubsystem, visionSubsystem));
        // Controls.mainDriveControls
        //     .getJoystickButtonOf(Controls.mainDriveControls.alignToRightReefUsingVision)
        //     // .whileTrue(new AlignToReefTagRelative(true, swerveSubsystem));
        //     .whileTrue(new PhotonVisionAlign(true, swerveSubsystem, visionSubsystem));

        // Align (new)
        // driverController
        //     .getJoystickButton(XboxController.Button.kX)
        //     .whileTrue(
        //         new DeferredCommand(
        //             () -> autoRoutines.pathFindToLocation(autoRoutines.getNearestCoralStation()),
        //             Set.of(swerveSubsystem)
        //         )
        //     );

        driverController
            .getJoystickButton(XboxController.Button.kA)
            .whileTrue(autoRoutines.pathFindToLocation(autoRoutines::getNearestRightReef));
        // driverController
        //     .getJoystickButton(XboxController.Button.kB)
        //     .whileTrue(autoRoutines.continuouslyPathFindToLocation(() -> autoRoutines.getNearestCoralStation()));

        // Claw
        operatorController
            .getJoystickButton(Controls.Claw.zeroEncoder)
            .onTrue(Commands.runOnce(() -> clawSubsystem.io.zeroEncoder(0)));
        operatorController
            .getJoystickButton(Controls.Superstructure.clawOuttakeForL4)
            .whileTrue(autoRoutines.doClawMovementsForL4());

        // Elevator
        operatorController
            .getJoystickButton(Controls.Elevator.zeroEncoder)
            .onTrue(Commands.runOnce(() -> elevatorSubsystem.io.zeroEncoder(0)));

        // elevatorSubsystem.whenElevatorIsAtBottom.onTrue(
        //     // Reset elevator positions when hitting the limit switches
        //     Commands.runOnce(() -> {
        //         elevatorSubsystem.io.zeroEncoder(0);
        //         elevatorSubsystem.io.resetSetpointToCurrentPosition();
        //     })
        // );

        // Superstructure
        operatorController
            .getPOVButton(Controller.POVButtonDirection.UP)
            .whileTrue(autoRoutines.setUpSuperstructure(SuperstructureConstants.Level.INITIAL_POSITION));
        operatorController
            .getPOVButton(Controller.POVButtonDirection.LEFT)
            .whileTrue(autoRoutines.setUpSuperstructure(SuperstructureConstants.Level.L2));
        operatorController
            .getPOVButton(Controller.POVButtonDirection.DOWN)
            .whileTrue(autoRoutines.setUpSuperstructure(SuperstructureConstants.Level.L3));
        operatorController
            .getPOVButton(Controller.POVButtonDirection.RIGHT)
            .whileTrue(autoRoutines.setUpSuperstructure(SuperstructureConstants.Level.L4));

        // Algae presets
        operatorController
            .getJoystickButton(Controls.Superstructure.intakeAlgaeFromL2)
            .whileTrue(autoRoutines.intakeAlgae(SuperstructureConstants.Level.ALGAE_L2));
        operatorController
            .getJoystickButton(Controls.Superstructure.intakeAlgaeFromL3)
            .whileTrue(autoRoutines.intakeAlgae(SuperstructureConstants.Level.ALGAE_L3));
        operatorController.getJoystickButton(Controls.Superstructure.launchAlgae).whileTrue(autoRoutines.launchAlgae());
    }

    public void assignDefaultCommands() {
        clawSubsystem.setDefaultCommand(
            clawSubsystem.getRunAndAngleCommand(Controls.Claw::getIntakeOrOuttake, Controls.Claw::getUpOrDown)
        );

        elevatorSubsystem.setDefaultCommand(elevatorSubsystem.getUpOrDown(Controls.Elevator::getUpOrDown));
    }
}
