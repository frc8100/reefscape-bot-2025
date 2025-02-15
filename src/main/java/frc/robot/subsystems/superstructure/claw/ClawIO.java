package frc.robot.subsystems.superstructure.claw;

import org.littletonrobotics.junction.AutoLog;

/**
 * The IO interface for the claw.
 */
public interface ClawIO {
    /**
     * The inputs that should be supplied by the IO interface.
     */
    @AutoLog
    public static class ClawIOInputs {
        public boolean connected = true;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    public void updateInputs(ClawIOInputs inputs);

    /**
     * Runs the claw motor.
     * @param motorInput - percent input, from [-1, 1] without deadband.
     */
    public void runClaw(double motorInput);

    /**
     * Stops the motor
     */
    public void stop();
}
