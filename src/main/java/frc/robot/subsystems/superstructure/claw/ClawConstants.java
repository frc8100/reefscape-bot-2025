package frc.robot.subsystems.superstructure.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;

/**
 * Constants for the claw. Includes CAN Ids.
 */
public final class ClawConstants {
    private ClawConstants() {}

    /**
     * Contains the angle measures for certain claw rotations
     */
    public static final class RotationPositions {
        private RotationPositions() {}

        public static final Rotation2d RESTING_ANGLE = new Rotation2d();
        public static final Rotation2d L4ANGLE = new Rotation2d(Degrees.of(10));
    }

    /**
     * Deadband for the claw (controller input).
     */
    public static final double ARM_CONTROLLER_DEADBAND = 0.1;

    /**
     * The percent power (from 0-1) to run the intake/outtake
     */
    public static final double ARM_MAX_OUTPUT = 0.15;

    // Turn motor configs
    public static final int ANGLE_MOTOR_ID = 12;
    public static final double ANGLE_GEAR_RATIO = 15.0;
    public static final boolean IS_ANGLE_MOTOR_INVERTED = false;
    public static final Current ANGLE_MOTOR_CURRENT_LIMIT = Amps.of(40);
    /** Rotations to radians */
    public static final double ANGLE_ENCODER_POSITION_FACTOR = (2 * Math.PI) / ANGLE_GEAR_RATIO;

    // PID configs
    public static final double ANGLE_KP = 0.1;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.0;
    public static final double ANGLE_KF = 0.0;

    /**
     * The maximum power for the angle motor, from 0-1.
     */
    public static final double MAX_ANGLE_POWER = 0.8;

    // Outtake motor configs
    public static final int OUTTAKE_MOTOR_ID = 13;
    public static final double OUTTAKE_GEAR_RATIO = 1.0;
    public static final boolean IS_OUTTAKE_MOTOR_INVERTED = false;
    public static final Current OUTTAKE_MOTOR_CURRENT_LIMIT = Amps.of(40);
    /** Rotations to radians */
    public static final double OUTTAKE_ENCODER_POSITION_FACTOR = (2 * Math.PI) / OUTTAKE_GEAR_RATIO;

    /**
     * The maximum power for the outtake motor, from 0-1.
     */
    public static final double MAX_OUTTAKE_POWER = 0.7;
}
