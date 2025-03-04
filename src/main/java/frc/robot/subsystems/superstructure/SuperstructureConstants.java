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
        L1(Meters.of(0.0), new Rotation2d(0)),
        L2(Meters.of(0.9).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(37.5).minus(CLAW_ANGLE_OFFSET)),
        L3(Meters.of(1.25).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(37.5).minus(CLAW_ANGLE_OFFSET)),
        L4(Meters.of(2.1).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(90).minus(CLAW_ANGLE_OFFSET));

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

        private Level(Distance elevatorDistance, Rotation2d clawAngle) {
            this.elevatorDistance = elevatorDistance;
            this.clawAngle = clawAngle;
        }
    }
}
