package frc.robot.subsystems.superstructure.claw;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
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

        /**
         * The initial angle of the claw when it is resting (against the mechanical lock).
         * The lower it is, the more "vertical" the claw is.
         */
        public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(45);

        public static final Rotation2d RESTING_ANGLE = Rotation2d.fromDegrees(0);

        public static final Rotation2d L4ANGLE = Rotation2d.fromDegrees(90).minus(ANGLE_OFFSET);
        public static final Rotation2d L3ANGLE = Rotation2d.fromDegrees(35).minus(ANGLE_OFFSET);
        public static final Rotation2d L2ANGLE = Rotation2d.fromDegrees(35).minus(ANGLE_OFFSET);
        public static final Rotation2d L1ANGLE = Rotation2d.fromDegrees(45).minus(ANGLE_OFFSET);
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
    public static final int ANGLE_MOTOR_ID = 9;
    /**
     * The maximum power for the angle motor, from 0-1.
     */
    // TODO: Tune
    public static final double MAX_ANGLE_POWER = 0.08;

    public static final double ANGLE_GEAR_RATIO = 30;
    public static final boolean IS_ANGLE_MOTOR_INVERTED = true;
    public static final Current ANGLE_MOTOR_CURRENT_LIMIT = Amps.of(40);
    /** Rotations to radians */
    public static final double ANGLE_ENCODER_POSITION_FACTOR = (2 * Math.PI) / ANGLE_GEAR_RATIO;

    // PID configs
    public static final double ANGLE_KP = 0.1;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.0;
    public static final double ANGLE_KF = 0.0;

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
    public static final double MAX_OUTTAKE_POWER = 0.5;

    // Simulator configs
    public static final DCMotor SIM_ANGLE_MOTOR = DCMotor.getNEO(1);
    public static final double SIM_ANGLE_MOI = 0.01;
    public static final double SIM_ANGLE_KP = 5.0;
    public static final double SIM_ANGLE_KI = 0.0;
    public static final double SIM_ANGLE_KD = 0.0;

    public static final DCMotor SIM_OUTTAKE_MOTOR = DCMotor.getNEO(1);
    public static final double SIM_OUTTAKE_MOI = 0.01;
}
