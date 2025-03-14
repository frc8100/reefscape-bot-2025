package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.superstructure.claw.ClawConstants.RotationPositions.CLAW_ANGLE_OFFSET;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.Position.INITIAL_HEIGHT_CLAW;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

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
        L1(Meters.of(0.0), CLAW_ANGLE_OFFSET, 0),
        L2(Meters.of(0.9).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(60), 4.8),
        L3(Meters.of(1.25).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(60), 12.35),
        // TODO: uh dont do this
        L4(Meters.of(2.1).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(95), 20),

        ALGAE_L1(Meters.of(0), Rotation2d.fromDegrees(160), 0);

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

        /**
         * @return The radian of the elevator.
         */
        public double getElevatorRadian() {
            return elevatorRadian;
        }

        /**
         * Creates a new level.
         * @param elevatorDistance - The distance to run the elevator.
         * @param clawAngle - The angle to run the claw, without the offset.
         */
        private Level(Distance elevatorDistance, Rotation2d clawAngle, double elevatorRadian) {
            this.elevatorDistance = elevatorDistance;
            this.clawAngle = clawAngle.minus(CLAW_ANGLE_OFFSET);
            this.elevatorRadian = elevatorRadian;
        }
    }
}
