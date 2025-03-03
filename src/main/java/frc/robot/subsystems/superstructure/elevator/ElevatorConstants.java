package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

/**
 * Constants for the elevator.
 */
public final class ElevatorConstants {

    private ElevatorConstants() {}

    /**
     * Position constants for the elevator.
     */
    public static final class Position {

        private Position() {}

        /**
         * The initial height of the claw.
         */
        public static final Distance INITIAL_HEIGHT_CLAW = Meters.of(0.5);

        // The positions of the elevator in meters
        public static final Distance L1_DISTANCE = Meters.of(0);
        public static final Distance L2_DISTANCE = Meters.of(0.9).minus(INITIAL_HEIGHT_CLAW);
        public static final Distance L3_DISTANCE = Meters.of(1.3).minus(INITIAL_HEIGHT_CLAW);
        public static final Distance L4_DISTANCE = Meters.of(2.1).minus(INITIAL_HEIGHT_CLAW);

        public static final Distance MIN_HEIGHT = Meters.of(0);
        public static final Distance MAX_HEIGHT = Meters.of(2.0);

        public static final Distance STAGE_1_HEIGHT = Meters.of(0.7);
        public static final Distance STAGE_1_MAX_HEIGHT = Meters.of(1.0);
        public static final Distance STAGE_2_HEIGHT = Meters.of(0.5);
    }

    // TODO: Tune these values
    // Elevator motor constants
    /**
     * The CAN ID of the elevator motor.
     */
    public static final int ELEVATOR_MOTOR_ID = 15;
    public static final double ELEVATOR_MAX_OUTPUT = 0.4;
    public static final double ELEVATOR_GEAR_RATIO = 1.0;
    public static final boolean ELEVATOR_MOTOR_INVERTED = false;
    public static final Current ELEVATOR_MOTOR_CURRENT_LIMIT = Amps.of(40);
    /** Rotations to radians */
    public static final double ELEVATOR_MOTOR_POSITION_FACTOR = (2 * Math.PI) / ELEVATOR_GEAR_RATIO;

    public static final LinearVelocity ELEVATOR_MAX_VELOCITY = MetersPerSecond.of(2.5);
    public static final LinearAcceleration ELEVATOR_MAX_ACCELERATION = MetersPerSecondPerSecond.of(3);

    // PID constants
    public static final double ELEVATOR_KP = 0.1;
    public static final double ELEVATOR_KI = 0.0;
    public static final double ELEVATOR_KD = 0.0;
    public static final double ELEVATOR_KF = 0.0;

    // TODO: Tune these values
    public static final Mass ELEVATOR_MASS = Kilograms.of(5.0);

    // Simulation constants
    public static final DCMotor SIM_MOTOR = DCMotor.getNEO(2);
    public static final MomentOfInertia SIM_MOI = KilogramSquareMeters.of(0.1);

    public static final double SIM_KP = 7.0;
    public static final double SIM_KI = 0.0;
    public static final double SIM_KD = 0.0;
    public static final double SIM_KF = 1.0;

    public static final Distance ELEVATOR_DRUM_RADIUS = Inches.of(2);

    public static final double SIM_KS = 0.0; // volts (V)
    public static final double SIM_KG = 0.762; // volts (V)
    public static final double SIM_KV = 0.762; // volt per velocity (V/(m/s))
    public static final double SIM_KA = 0.0; // volt per acceleration (V/(m/s²))
}
