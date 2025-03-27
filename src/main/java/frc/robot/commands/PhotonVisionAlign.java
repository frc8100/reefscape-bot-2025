package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.TunableValue;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import org.photonvision.PhotonUtils;

/**
 * Aligns the robot to a target using PhotonVision and PID control.
 */
public class PhotonVisionAlign extends Command {

    private static final String DASHBOARD_STRING = "AlignP/";

    private static double ROT_SETPOINT_REEF_ALIGNMENT = 0; // Rotation

    private static double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
    private static double X_SETPOINT_REEF_ALIGNMENT = -0.68; // TZ / Vertical pose -0.16

    private static double X_TOLERANCE_REEF_ALIGNMENT = 0.04;
    private static double Y_SETPOINT_REEF_ALIGNMENT = 0.31; // Horizontal pose

    private static double Y_TOLERANCE_REEF_ALIGNMENT = -0.15;

    private static final TunableValue ROT_SETPOINT_REEF_ALIGNMENT_TUNABLE = new TunableValue(
        DASHBOARD_STRING + "RotationSetpoint",
        ROT_SETPOINT_REEF_ALIGNMENT,
        (double value) -> ROT_SETPOINT_REEF_ALIGNMENT = value
    );
    private static final TunableValue X_SETPOINT_REEF_ALIGNMENT_TUNABLE = new TunableValue(
        DASHBOARD_STRING + "XSetpoint",
        X_SETPOINT_REEF_ALIGNMENT,
        (double value) -> X_SETPOINT_REEF_ALIGNMENT = value
    );
    private static final TunableValue Y_SETPOINT_REEF_ALIGNMENT_TUNABLE = new TunableValue(
        DASHBOARD_STRING + "YSetpoint",
        Y_SETPOINT_REEF_ALIGNMENT,
        (double value) -> Y_SETPOINT_REEF_ALIGNMENT = value
    );

    private static double X_P = 0.85;
    private static double X_D = 0.005;

    private static double Y_P = 0.75;
    private static double Y_D = 0.005;

    private static double R_P = 0.04;
    private static double R_D = 0.002;

    private static final TunableValue X_P_TUNABLE = new TunableValue(DASHBOARD_STRING + "XP", X_P, (double value) ->
        X_P = value
    );
    private static final TunableValue X_D_TUNABLE = new TunableValue(DASHBOARD_STRING + "XD", X_D, (double value) ->
        X_D = value
    );

    private static final TunableValue Y_P_TUNABLE = new TunableValue(DASHBOARD_STRING + "YP", Y_P, (double value) ->
        Y_P = value
    );
    private static final TunableValue Y_D_TUNABLE = new TunableValue(DASHBOARD_STRING + "YD", Y_D, (double value) ->
        Y_D = value
    );

    private static final TunableValue R_P_TUNABLE = new TunableValue(DASHBOARD_STRING + "RP", R_P, (double value) ->
        R_P = value
    );
    private static final TunableValue R_D_TUNABLE = new TunableValue(DASHBOARD_STRING + "RD", R_D, (double value) ->
        R_D = value
    );

    private SwerveDrive swerveSubsystem;
    private Vision visionSubsystem;

    public PhotonVisionAlign(SwerveDrive swerveSubsystem, Vision visionSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;
    }
}
