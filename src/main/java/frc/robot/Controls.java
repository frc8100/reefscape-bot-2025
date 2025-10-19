package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.superstructure.claw.ClawConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

/** Declares control key bindings */
public class Controls {

    private Controls() {}

    /**
     * The driver controller, on port 0. Note: although this uses the {@link Joystick} class, it is compatible with {@link XboxController}.
     */
    public static final Joystick mainDriverController = new Joystick(0);

    public static final Drive mainDriveControls = new Drive(mainDriverController);
    public static final Drive joystickDriveControls = new JoystickDrive(Controls.mainDriverController);

    /**
     * Whether or not to use the joystick drive.
     */
    public static final boolean isUsingJoystickDrive = false;

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

            robotRelativeMoveForward = new POVButton(driverController, 0);
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
        public int zeroGyroButton = XboxController.Button.kY.value;

        /**
         * When held, dampens the robot movement. This will decrease the robot's speed a lot.
         */
        public int dampenButton = XboxController.Button.kRightBumper.value;
        /**
         * When held, slows the robot down to {@link #slowMultiplier}
         */
        public int slowButton = XboxController.Button.kLeftBumper.value;

        public double slowMultiplier = 0.7;

        /**
         * When held, makes the robot move robot centric.
         */
        public int robotCentricButton = XboxController.Button.kX.value;

        /**
         * Aligns the robot to the reef.
         */
        public int alignToLeftReefUsingVision = XboxController.Button.kB.value;
        public int alignToRightReefUsingVision = XboxController.Button.kX.value;

        /**
         * Aligns the robot to the nearest left reef.
         */
        // public int pathfindToNearestLeftReef = XboxController.Button.kB.value;

        /**
         * Aligns the robot to the nearest right reef.
         */
        // public int pathfindToNearestRightReef = XboxController.Button.kA.value;

        // public int pathfindToNearestCoralStation = XboxController.Button.kX.value;

        public POVButton robotRelativeMoveForward;

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
            // TODO
            // return isCurrentlyRobotCentric;
            return false;
        }

        /**
         * @return Whether the robot movement is dampened.
         */
        public boolean isDampen() {
            return driverController.getRawButton(dampenButton);
        }

        /**
         * @return The speed multiplier.
         */
        public double getSpeedMultiplier() {
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
        public static final Joystick clawController = new Joystick(1);

        // public static final JoystickButton

        // Buttons for the intake/outtake
        private static final int intakeBackAxis = XboxController.Axis.kLeftTrigger.value;
        private static final int intakeOutAxis = XboxController.Axis.kRightTrigger.value;

        /**
         * Returns the intake or outtake input depending on which one is bigger.
         * This does not include deadband.
         * @return A double from [-1, 1]. If it is outtake, it will be negative, otherwise it will be positive.
         */
        public static final double getIntakeOrOuttake() {
            // Get controller input
            double intakeInput = clawController.getRawAxis(intakeBackAxis);
            double outtakeInput = clawController.getRawAxis(intakeOutAxis);

            // Get the larger of them
            if (intakeInput >= outtakeInput) {
                return ClawConstants.IntakeOuttakeDirection.BACK.getDirection() * intakeInput;
            } else {
                return ClawConstants.IntakeOuttakeDirection.OUTTAKE.getDirection() * outtakeInput;
            }
        }

        public static final int upButton = XboxController.Button.kB.value;
        public static final int downButton = XboxController.Button.kA.value;

        public static final int upDownAxis = XboxController.Axis.kRightY.value;

        /**
         * Amount to increase by per frame
         */
        public static final double clawIncreaseAmount = 0.0325;

        public static final double slowAmount = 0.4;

        public static final double getUpOrDown() {
            return -clawController.getRawAxis(upDownAxis) * (clawController.getRawButton(slowButton) ? slowAmount : 1);
            // Get controller input
            // double intakeInput = clawController.getRawAxis(upDownAxis);
            // double outtakeInput = clawController.getRawAxis(outtakeAxis);

            // // Get the larger of them
            // if (intakeInput >= outtakeInput) {
            //     return 1;
            // } else {
            //     return -1;
            // }
        }

        public static final int zeroEncoder = XboxController.Button.kLeftBumper.value;
        public static final int slowButton = XboxController.Button.kRightBumper.value;
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
        public static final Joystick elevatorController = Claw.clawController;

        public static final int zeroEncoder = XboxController.Button.kLeftBumper.value;

        public static final int slowButton = XboxController.Button.kRightBumper.value;
        public static final double slowAmount = 0.4;

        private static final int upDownAxis = XboxController.Axis.kLeftY.value;

        public static final double getUpOrDown() {
            return (
                -elevatorController.getRawAxis(upDownAxis) *
                (elevatorController.getRawButton(slowButton) ? slowAmount : 1)
            );
            // Get controller input
            // double intakeInput = clawController.getRawAxis(upDownAxis);
            // double outtakeInput = clawController.getRawAxis(outtakeAxis);

            // // Get the larger of them
            // if (intakeInput >= outtakeInput) {
            //     return 1;
            // } else {
            //     return -1;
            // }
        }
    }

    /**
     * The controls for the superstructure.
     */
    public static final class Superstructure {

        private Superstructure() {}

        /**
         * The controller used for the superstructure.
         * Temporarily the same as the driver controller.
         */
        public static final Joystick superstructureController = Claw.clawController;

        /**
         * The button to make the superstructure go to L4. Default is right on the D-pad.
         */
        public static final POVButton moveToL4 = new POVButton(superstructureController, 90);

        /**
         * The button to make the superstructure go to L3. Default is down on the D-pad.
         */
        public static final POVButton moveToL3 = new POVButton(superstructureController, 180);

        /**
         * The button to make the superstructure go to L2. Default is left on the D-pad.
         */
        public static final POVButton moveToL2 = new POVButton(superstructureController, 270);

        /**
         * The button to make the superstructure go to L1. Default is up on the D-pad.
         */
        public static final POVButton moveToL1 = new POVButton(superstructureController, 0);

        /**
         * The button to intake algae from L2
         */
        public static final int intakeAlgaeFromL2 = XboxController.Button.kA.value;
        public static final int intakeAlgaeFromL3 = XboxController.Button.kB.value;

        public static final int clawOuttakeForL4 = XboxController.Button.kY.value;

        public static final int launchAlgae = XboxController.Button.kX.value;
    }
}
