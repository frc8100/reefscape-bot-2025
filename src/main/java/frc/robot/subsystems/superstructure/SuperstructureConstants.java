package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.superstructure.claw.ClawConstants.RotationPositions.CLAW_ANGLE_OFFSET;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.Position.INITIAL_HEIGHT_CLAW;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.TunableValue;

/**
 * Contains the constants for the superstructure.
 */
public final class SuperstructureConstants {

    /**
     * The levels of the reef and the corresponding elevator height and claw angle
     */
    public enum Level {
        /**
         * Also the initial position of the elevator and claw.
         */
        INITIAL_POSITION("L1", Meters.of(0.0), CLAW_ANGLE_OFFSET, 0),
        L2("L2", Meters.of(0.9).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(60), 5.3),
        L3("L3", Meters.of(1.25).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(60), 14.9),
        L4("L4", Meters.of(2.1).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(95), 22),

        L1_AUTO("L1Auto", Meters.of(0.4).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(180), 3.5),

        ALGAE_L2("L2Algae", Meters.of(0), Rotation2d.fromDegrees(180), 4.8);

        private final Distance elevatorDistance;

        /**
         * @return The distance of the elevator.
         * Measured from the ground to the top of the elevator.
         */
        public Distance getElevatorDistance() {
            return elevatorDistance;
        }

        private final Rotation2d clawAngle;

        /**
         * @return The angle of the claw.
         */
        public Rotation2d getClawAngle() {
            return clawAngle;
        }

        private final double elevatorRadian;
        private final TunableValue elevatorRadianTunable;

        /**
         * @return The radian of the elevator.
         */
        public double getElevatorRadian() {
            // return elevatorRadian;

            return elevatorRadianTunable.get();
        }

        /**
         * Creates a new level.
         * @param elevatorDistance - The distance to run the elevator.
         * @param clawAngle - The angle to run the claw, without the offset.
         */
        private Level(String key, Distance elevatorDistance, Rotation2d clawAngle, double elevatorRadian) {
            this.elevatorDistance = elevatorDistance;
            this.clawAngle = clawAngle.minus(CLAW_ANGLE_OFFSET);
            this.elevatorRadian = elevatorRadian;

            this.elevatorRadianTunable = new TunableValue("SSLevels/" + key, elevatorRadian);
        }
    }
}
