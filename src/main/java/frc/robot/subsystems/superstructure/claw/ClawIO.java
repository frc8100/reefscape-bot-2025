package frc.robot.subsystems.superstructure.claw;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.GenericSparkIO;
import org.littletonrobotics.junction.AutoLog;

/**
 * The IO interface for the claw.
 */
public interface ClawIO extends GenericSparkIO<ClawIO.ClawIOInputs> {
    /**
     * The inputs that should be supplied by the IO interface.
     */
    @AutoLog
    public static class ClawIOInputs {
        // Inputs from the turn motor
        public boolean turnConnected = true;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnSupplyCurrentAmps = 0.0;
        public double turnTorqueCurrentAmps = 0.0;
        public double turnTempCelsius = 0.0;

        // Inputs from the outake motor
        public boolean outakeConnected = true;
        public double outakePositionRad = 0.0;
        public double outakeVelocityRadPerSec = 0.0;
        public double outakeAppliedVolts = 0.0;
        public double outakeSupplyCurrentAmps = 0.0;
        public double outakeTorqueCurrentAmps = 0.0;
        public double outakeTempCelsius = 0.0;
    }

    /**
     * Stops both motors.
     */
    public void stop();

    /**
     * Sets the desired turn position.
     * @param rotation - The desired turn position.
     */
    public void setTurnPosition(Rotation2d rotation);

    /**
     * Runs the outake motor at the specified percent output.
     * @param motorInput - The percent output to run the motor at, from [-1, 1] without deadband.
     */
    public void runOutake(double motorInput);

    /**
     * Periodic function to be called by the subsystem.
     * Optional, do not update the IO here.
     */
    public default void periodic() {}
}
