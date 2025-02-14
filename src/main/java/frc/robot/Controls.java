package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Declares control key bindings */
public class Controls {

    /**
     * The driver controller, on port 0. Note: although this uses the {@link Joystick} class, it is compatible with {@link XboxController}.
     */
    public static final Joystick driverController = new Joystick(0);

    /** The up controller, on port 1 */
    // public static final XboxController upController = new XboxController(1);
    // public static final CommandXboxController upController = new CommandXboxController(1);

    /** The drive controls */
    public static class Drive {
        /** Whether to invert the drive controls. Default is `true`. */
        public static final boolean invertDriveControls = true;

        // Driver Controls
        // By default, the left stick controls robot movement (translation - y, strafe - x)
        // and the right stick controls the rotation (x)
        private static final int translationAxis = XboxController.Axis.kLeftY.value;
        private static final int strafeAxis = XboxController.Axis.kLeftX.value;
        private static final int rotationAxis = XboxController.Axis.kRightX.value;

        // Driver Buttons
        /**
         * When pressed, zeroes the gyro. Default is {@link XboxController.Button#kY} (top button).
         * Press when robot is facing towards the drive station to align the robot's forward direction with the field.
         */
        public static final JoystickButton zeroGyro =
                new JoystickButton(driverController, XboxController.Button.kY.value);

        /**
         * When held, dampens the robot movement. This will decrease the robot's speed a lot.
         */
        public static final JoystickButton dampen =
                new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
        /**
         * When held, slows the robot down to {@link #slowMultiplier}
         */
        public static final JoystickButton slowButton =
                new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);

        public static final double slowMultiplier = 0.5;

        // unused
        public static final JoystickButton robotCentric =
                new JoystickButton(driverController, XboxController.Button.kB.value);

        // Direction buttons
        public static final POVButton up = new POVButton(driverController, 90);
        public static final POVButton down = new POVButton(driverController, 270);
        public static final POVButton right = new POVButton(driverController, 180);
        public static final POVButton left = new POVButton(driverController, 0);

        // TODO: give more descriptive name
        public static final JoystickButton goToCoralStation1 =
                new JoystickButton(driverController, XboxController.Button.kA.value);

        public static final JoystickButton goToReef1 =
                new JoystickButton(driverController, XboxController.Button.kB.value);

        /**
         * @return The translation (x)
         */
        public static double getTranslationAxis() {
            return invertDriveControls
                    ? -driverController.getRawAxis(translationAxis)
                    : driverController.getRawAxis(translationAxis);
        }

        /**
         * @return The strafe (y)
         */
        public static double getStrafeAxis() {
            return invertDriveControls
                    ? -driverController.getRawAxis(strafeAxis)
                    : driverController.getRawAxis(strafeAxis);
        }

        /**
         * @return The rotation
         */
        public static double getRotationAxis() {
            return invertDriveControls
                    ? -driverController.getRawAxis(rotationAxis)
                    : driverController.getRawAxis(rotationAxis);
        }

        /**
         * @return Whether the controls are robot centric. Default is `false`.
         */
        public static boolean isRobotCentric() {
            // return false;
            return robotCentric.getAsBoolean();
        }

        /**
         * @return Whether the robot movement is dampened.
         */
        public static boolean isDampen() {
            return Drive.dampen.getAsBoolean();
        }

        /**
         * @return The speed multiplier.
         */
        public static double getSpeedMultiplier() {
            return slowButton.getAsBoolean() ? slowMultiplier : 1;
        }
    }

    /**
     * The controls for the arm.
     */
    public static class Arm {
        /**
         * The controller used for the arm.
         * Temporarily the same as the driver controller.
         */
        public static final Joystick armController = driverController;

        // public static final JoystickButton

        // Buttons for the intake/outtake
        private static final int intakeAxis = XboxController.Axis.kLeftTrigger.value;
        private static final int outtakeAxis = XboxController.Axis.kRightTrigger.value;

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
}
