package frc.robot;

/**
 * Drive states
 */
public class States {

    /**
     * Possible drive states
     */
    public static enum DriveStates {
        standard,
        d0,
        d90,
        d180,
        d270,
    }

    /**
     * The current drive state
     */
    public static DriveStates driveState = DriveStates.standard;
}
