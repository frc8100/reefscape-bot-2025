package frc.robot.subsystems.superstructure.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

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
         * The lower it is, the more "horizontal" the claw is.
         */
        public static final Rotation2d CLAW_ANGLE_OFFSET = Rotation2d.fromDegrees(35);

        public static final Rotation2d RESTING_ANGLE = Rotation2d.fromDegrees(0);

        /**
         * The horizontal, front/back translation of the claw from the origin of the second elevator stage.
         */
        public static final double ELEVATOR_TO_CLAW_X = 0.27;

        /**
         * The vertical, up/down translation of the claw from the origin of the second elevator stage.
         */
        public static final double ELEVATOR_TO_CLAW_Z = 0.37;

        /**
         * The translation from the origin of the claw to where the coral is stored.
         */
        private static final double CLAW_TO_CORAL = 0.3;

        /**
         * @return The translation (x only) from the origin of the claw to where the coral is stored.
         * @param angleRadians - The angle of the claw, in radians, with the offset applied.
         */
        public static Translation2d getClawToCoralX(double angleRadians) {
            // Undo the offset
            angleRadians = angleRadians + CLAW_ANGLE_OFFSET.getRadians();

            return new Translation2d(CLAW_TO_CORAL * Math.sin(angleRadians - (Math.PI / 4)), 0);
        }

        /**
         * @return The z (vertical) translation (in meters) from the origin of the claw to where the coral is stored.
         * @param angleRadians - The angle of the claw, in radians, with the offset applied.
         */
        public static double getClawToCoralZ(double angleRadians) {
            // Undo the offset
            angleRadians = angleRadians + CLAW_ANGLE_OFFSET.getRadians();

            return CLAW_TO_CORAL * Math.cos(angleRadians - (Math.PI / 4));
        }
    }

    /**
     * The direction of the intake/outtake.
     */
    public enum IntakeOuttakeDirection {
        INTAKE(1),
        OUTTAKE(-1);

        /**
         * The direction of the intake/outtake.
         * If the direction is 1, the intake/outtake is in.
         * If the direction is -1, the intake/outtake is out.
         */
        private final int direction;

        /**
         * @return The direction of the intake/outtake.
         */
        public int getDirection() {
            return direction;
        }

        /**
         * Creates a new IntakeOuttakeDirection based on the given direction.
         */
        IntakeOuttakeDirection(int direction) {
            this.direction = direction;
        }
    }

    /**
     * How long to run the intake for.
     */
    public static final Time INTAKE_TIME = Seconds.of(0.4);

    /**
     * How long to run the outtake for.
     */
    public static final Time OUTTAKE_TIME = Seconds.of(0.8);

    /**
     * How much the claw can be off from the desired angle before it is considered "in position".
     * This is in radians.
     */
    public static final Angle ANGLE_TOLERANCE_RADIANS = Degrees.of(2);

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

    // -0.16

    // Outtake motor configs
    public static final int OUTTAKE_MOTOR_ID = 15;
    public static final double OUTTAKE_GEAR_RATIO = 5;
    public static final boolean IS_OUTTAKE_MOTOR_INVERTED = false;
    public static final Current OUTTAKE_MOTOR_CURRENT_LIMIT = Amps.of(40);
    /** Rotations to radians */
    public static final double OUTTAKE_ENCODER_POSITION_FACTOR = (2 * Math.PI) / OUTTAKE_GEAR_RATIO;

    /**
     * The maximum power for the outtake motor, from 0-1.
     */
    public static final double MAX_OUTTAKE_POWER = 0.3;

    // Simulator configs
    public static final DCMotor SIM_ANGLE_MOTOR = DCMotor.getNEO(1);
    public static final MomentOfInertia SIM_ANGLE_MOI = KilogramSquareMeters.of(0.1);
    public static final Voltage SIM_ANGLE_FRICTION_VOLTAGE = Volts.of(0.1);
    public static final double SIM_ANGLE_KP = 4.0;
    public static final double SIM_ANGLE_KI = 0.0;
    public static final double SIM_ANGLE_KD = 0.0;

    public static final SimMotorConfigs SIM_ANGLE_MOTOR_CONFIG = new SimMotorConfigs(
        SIM_ANGLE_MOTOR,
        ANGLE_GEAR_RATIO,
        SIM_ANGLE_MOI,
        SIM_ANGLE_FRICTION_VOLTAGE
    );

    public static final DCMotor SIM_OUTTAKE_MOTOR = DCMotor.getNEO(1);
    public static final AngularVelocity SIM_OUTTAKE_TARGET_VELOCITY = RadiansPerSecond.of(25);
    public static final MomentOfInertia SIM_OUTTAKE_MOI = KilogramSquareMeters.of(0.05);
    public static final Voltage SIM_OUTTAKE_FRICTION_VOLTAGE = Volts.of(0.1);
    public static final double SIM_OUTTAKE_KP = 0.6;
    public static final double SIM_OUTTAKE_KI = 0.0;
    public static final double SIM_OUTTAKE_KD = 0.0;

    public static final SimMotorConfigs SIM_OUTTAKE_MOTOR_CONFIG = new SimMotorConfigs(
        SIM_OUTTAKE_MOTOR,
        OUTTAKE_GEAR_RATIO,
        SIM_OUTTAKE_MOI,
        SIM_OUTTAKE_FRICTION_VOLTAGE
    );

    /**
     * The speed that the coral is ejected at.
     */
    public static final LinearVelocity SIM_OUTTAKE_EJECT_SPEED = MetersPerSecond.of(1.75);
}
