package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.units.measure.Distance;
import frc.lib.util.GenericSparkIO;
import frc.lib.util.GenericSparkIO.GenericSparkIOInputs;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.junction.AutoLog;

/**
 * The IO interface for the elevator.
 */
public interface ElevatorIO extends GenericSparkIO<ElevatorIO.ElevatorIOInputs> {
    /**
     * The inputs that should be supplied by the IO interface.
     */
    @AutoLog
    public static class ElevatorIOInputs extends GenericSparkIOInputs {

        // public boolean isAtBottom = false;
        // public boolean isAtTop = false;
        // public double positionMeters = 0.0;

        /**
         * The height of the elevator in meters.
         */
        public double height = 0.0;

        /**
         * The setpoint of the elevator in meters.
         */
        public double setpoint = 0.0;

        /**
         * The velocity of the elevator in meters per second.
         */
        public double velocity = 0.0;

        /**
         * The setpoint of the elevator in meters per second.
         */
        public double setpointVelocity = 0.0;
    }

    /**
     * Stops the elevator.
     */
    public void stop();

    /**
     * Sets the desired elevator position.
     * @param position - The desired elevator position.
     */
    public void setPosition(Distance position);

    /**
     * Sets the desired elevator position given a level. Can be used to get the radian position for SPARK io.
     * @param level - The level to set the elevator to. See {@link SuperstructureConstants.Level} for available levels.
     */
    public default void setPosition(SuperstructureConstants.Level level) {
        setPosition(level.getElevatorDistance());
    }

    public default void resetSetpointToCurrentPosition() {}

    /**
     * Periodic function to be called by the subsystem.
     * Optional, do not update the IO here.
     */
    public default void periodic() {}

    /**
     * Runs the elevator motor at the given input.
     * @param motorInput - The input to the motor from -1 to 1.
     */
    public default void runMotor(double motorInput) {}

    /**
     * Zeros the elevator encoder.
     * @param value - The value to set the encoder to.
     */
    public default void zeroEncoder(double value) {}
}
