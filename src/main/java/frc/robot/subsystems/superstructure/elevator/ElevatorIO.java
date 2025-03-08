package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.units.measure.Distance;
import frc.lib.util.GenericSparkIO;
import frc.lib.util.GenericSparkIO.GenericSparkIOInputs;
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
     * Periodic function to be called by the subsystem.
     * Optional, do not update the IO here.
     */
    public default void periodic() {}

    public default void runMotor(double motorInput) {}
}
