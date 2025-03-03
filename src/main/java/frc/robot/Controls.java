package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Declares control key bindings */
public class Controls {

    private Controls() {}

    /**
     * The driver controller, on port 0. Note: although this uses the {@link Joystick} class, it is compatible with {@link XboxController}.
     */
    public static final Joystick mainDriverController = new Joystick(0);

    public static final Drive mainDriveControls = new Drive(mainDriverController);

    /** The drive controls */
    public static class Drive {

        private Joystick driverController;

        /**
         * Creates a new drive controls object.
         * @param driverController The driver controller
         */
        public Drive(Joystick driverController) {
            this.driverController = driverController;

            // Register toggle modes
            getJoystickButtonOf(robotCentricButton).onTrue(
                Commands.runOnce(() -> isCurrentlyRobotCentric = !isCurrentlyRobotCentric)
            );
        }

        // Toggle modes
        public boolean isCurrentlyRobotCentric = false;

        /**
         * @return A new {@link JoystickButton} for the given button number
         */
        public JoystickButton getJoystickButtonOf(int button) {
            return new JoystickButton(driverController, button);
        }

        /** Whether to invert the drive controls. Default is `true`. */
        public boolean invertDriveControls = true;

        // Driver Controls
        // By default, the left stick controls robot movement (translation - y, strafe - x)
        // and the right stick controls the rotation (x)
        public int translationAxis = XboxController.Axis.kLeftY.value;
        public int strafeAxis = XboxController.Axis.kLeftX.value;
        public int rotationAxis = XboxController.Axis.kRightX.value;

        // Driver Buttons
        /**
         * When pressed, zeroes the gyro. Default is {@link XboxController.Button#kY} (top button).
         * Press when robot is facing towards the drive station to align the robot's forward direction with the field.
         */
        // public JoystickButton zeroGyro =
        //         new JoystickButton(driverController, XboxController.Button.kY.value);
        public int zeroGyroButton = XboxController.Button.kY.value;

        /**
         * When held, dampens the robot movement. This will decrease the robot's speed a lot.
         */
        // public JoystickButton dampen =
        //         new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
        public int dampenButton = XboxController.Button.kRightBumper.value;
        /**
         * When held, slows the robot down to {@link #slowMultiplier}
         */
        // public JoystickButton slowButton =
        //         new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
        public int slowButton = XboxController.Button.kLeftBumper.value;

        public double slowMultiplier = 0.5;

        /**
         * When held, makes the robot move robot centric.
         */
        // public JoystickButton robotCentric =
        //         new JoystickButton(driverController, XboxController.Button.kB.value);
        public int robotCentricButton = XboxController.Button.kX.value;

        // Direction buttons
        // public POVButton up = new POVButton(driverController, 90);
        // public POVButton down = new POVButton(driverController, 270);
        // public POVButton right = new POVButton(driverController, 180);
        // public POVButton left = new POVButton(driverController, 0);

        // ! Test
        // public JoystickButton goToCoralStation1 =
        //         new JoystickButton(driverController, XboxController.Button.kA.value);
        // public int goToCoralStation1Button = XboxController.Button.kA.value;

        // public JoystickButton goToReef1 =
        //         new JoystickButton(driverController, XboxController.Button.kB.value);
        // public int goToReef1Button = XboxController.Button.kB.value;

        /**
         * @return The translation (x)
         */
        public double getTranslationAxis() {
            return invertDriveControls
                ? -driverController.getRawAxis(translationAxis)
                : driverController.getRawAxis(translationAxis);
        }

        /**
         * @return The strafe (y)
         */
        public double getStrafeAxis() {
            return invertDriveControls
                ? -driverController.getRawAxis(strafeAxis)
                : driverController.getRawAxis(strafeAxis);
        }

        /**
         * @return The rotation
         */
        public double getRotationAxis() {
            return invertDriveControls
                ? -driverController.getRawAxis(rotationAxis)
                : driverController.getRawAxis(rotationAxis);
        }

        /**
         * @return Whether the controls are robot centric. Default is `false`.
         */
        public boolean isRobotCentric() {
            // return false;
            // return robotCentric.getAsBoolean();
            // return driverController.getRawButton(robotCentricButton);
            return isCurrentlyRobotCentric;
        }

        /**
         * @return Whether the robot movement is dampened.
         */
        public boolean isDampen() {
            // return dampen.getAsBoolean();
            return driverController.getRawButton(dampenButton);
        }

        /**
         * @return The speed multiplier.
         */
        public double getSpeedMultiplier() {
            // return slowButton.getAsBoolean() ? slowMultiplier : 1;
            return driverController.getRawButton(slowButton) ? slowMultiplier : 1;
        }
    }

    /**
     * Drive controls using a joystick
     */
    public static class JoystickDrive extends Drive {

        public JoystickDrive(Joystick driverController) {
            super(driverController);
            // Override axis values
            translationAxis = Joystick.AxisType.kY.value;
            strafeAxis = Joystick.AxisType.kX.value;
            rotationAxis = Joystick.AxisType.kZ.value;
        }
    }

    /**
     * The controls for the claw.
     */
    public static final class Claw {

        private Claw() {}

        /**
         * The controller used for the claw.
         * Temporarily the same as the driver controller.
         */
        public static final Joystick armController = mainDriverController;

        // public static final JoystickButton

        // Buttons for the intake/outtake
        private static final int intakeAxis = XboxController.Axis.kLeftTrigger.value;
        private static final int outtakeAxis = XboxController.Axis.kRightTrigger.value;

        /**
         * The button to make the claw rotated at L4. Default is up on the D-pad.
         */
        public static final POVButton moveToL4 = new POVButton(armController, 90);

        /**
         * The button to make the claw rotated at L3. Default is right on the D-pad.
         */
        public static final POVButton moveToL3 = new POVButton(armController, 180);

        /**
         * The button to make the claw rotated at L2. Default is down on the D-pad.
         */
        public static final POVButton moveToL2 = new POVButton(armController, 270);

        /**
         * The button to make the claw rotated at L1. Default is left on the D-pad.
         */
        public static final POVButton moveToL1 = new POVButton(armController, 0);

        /**
         * The button to make the claw rotated at the resting position. Default is A.
         */
        // public static final JoystickButton resetClawButton = new JoystickButton(
        //     armController,
        //     XboxController.Button.kA.value
        // );

        /**
         * Returns the intake or outtake input depending on which one is bigger.
         * This does not include deadband.
         * @return A double from [-1, 1]. If it is outtake, it will be negative, otherwise it will be positive.
         */
        public static final double getIntakeOrOuttake() {
            // Get controller input
            double intakeInput = armController.getRawAxis(intakeAxis);
            double outtakeInput = armController.getRawAxis(outtakeAxis);

            // Get the larger of them
            if (intakeInput >= outtakeInput) {
                return intakeInput;
            } else {
                return -outtakeInput;
            }
        }
    }

    /**
     * The controls for the elevator.
     */
    public static final class Elevator {

        private Elevator() {}

        /**
         * The controller used for the elevator.
         * Temporarily the same as the driver controller.
         */
        public static final Joystick elevatorController = mainDriverController;

        /**
         * The button to make the elevator go to L4. Default is up on the D-pad.
         */
        public static final POVButton moveToL4 = new POVButton(elevatorController, 90);

        /**
         * The button to make the elevator go to L3. Default is right on the D-pad.
         */
        public static final POVButton moveToL3 = new POVButton(elevatorController, 180);

        /**
         * The button to make the elevator go to L2. Default is down on the D-pad.
         */
        public static final POVButton moveToL2 = new POVButton(elevatorController, 270);

        /**
         * The button to make the elevator go to L1. Default is left on the D-pad.
         */
        public static final POVButton moveToL1 = new POVButton(elevatorController, 0);
    }
}
