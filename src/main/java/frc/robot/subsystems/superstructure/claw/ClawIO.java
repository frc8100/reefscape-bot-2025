package frc.robot.subsystems.superstructure.claw;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.util.GenericSparkIO;

/**
 * The IO interface for the claw.
 */
public interface ClawIO extends GenericSparkIO<ClawIO.ClawIOInputs> {
    /**
     * The inputs that should be supplied by the IO interface.
     */
    @AutoLog
    public static class ClawIOInputs extends GenericSparkIO.GenericSparkIOInputs {}
}
