package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.lib.util.TunableValue;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

/**
 * Aligns the robot to a target using PhotonVision and PID control.
 */
public class PhotonVisionAlign extends Command {

    private static final String DASHBOARD_STRING = "AlignPV/";

    private static double ROT_SETPOINT_REEF_ALIGNMENT = 0; // Rotation
    private static double ROT_RIGHT_SETPOINT_REEF_ALIGNMENT = 0.7;
    private static double ROT_TOLERANCE_REEF_ALIGNMENT = 1;

    private static double X_SETPOINT_REEF_ALIGNMENT = -0.68; // TZ / Vertical pose -0.16
    private static double X_RIGHT_SETPOINT_REEF_ALIGNMENT = -0.17;
    private static double X_TOLERANCE_REEF_ALIGNMENT = 0.04;

    private static double Y_SETPOINT_REEF_ALIGNMENT = 0.31; // Horizontal pose
    private static double Y_RIGHT_SETPOINT_REEF_ALIGNMENT = 1;
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

    private static final double DONT_SEE_TAG_WAIT_TIME = 1;
    private static final double POSE_VALIDATION_TIME = 0.3;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;

    private SwerveDrive swerveSubsystem;
    private Vision visionSubsystem;

    private boolean isRightScore;
    private Timer dontSeeTagTimer;
    private Timer stopTimer;

    private int tagID = -1;
    private double tagHeightMeters = 0.0;

    public PhotonVisionAlign(boolean isRightScore, SwerveDrive swerveSubsystem, Vision visionSubsystem) {
        this.isRightScore = isRightScore;
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(swerveSubsystem);

        xController = new PIDController(X_P_TUNABLE.get(), 0.0, X_D_TUNABLE.get());
        yController = new PIDController(Y_P_TUNABLE.get(), 0.0, Y_D_TUNABLE.get());
        rotController = new PIDController(R_P_TUNABLE.get(), 0.0, R_D_TUNABLE.get());

        // Refresh config
        TunableValue.addRefreshConfigConsumer(() -> {
            xController.setP(X_P_TUNABLE.get());
            xController.setD(X_D_TUNABLE.get());

            yController.setP(Y_P_TUNABLE.get());
            yController.setD(Y_D_TUNABLE.get());

            rotController.setP(R_P_TUNABLE.get());
            rotController.setD(R_D_TUNABLE.get());
        });
    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        // driveController.calculateRobotRelativeSpeeds(null, null)

        // driveController.setTolerance(
        //     new Pose2d(
        //         new Translation2d(X_TOLERANCE_REEF_ALIGNMENT, Y_TOLERANCE_REEF_ALIGNMENT),
        //         new Rotation2d(Y_TOLERANCE_REEF_ALIGNMENT)
        //     )
        // );

        // driveController.calculate(driveSubsystem.getPose(), new Trajectory.State(), null);

        rotController.setSetpoint(!isRightScore ? ROT_SETPOINT_REEF_ALIGNMENT : ROT_RIGHT_SETPOINT_REEF_ALIGNMENT);
        rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);

        xController.setSetpoint(!isRightScore ? X_SETPOINT_REEF_ALIGNMENT : X_RIGHT_SETPOINT_REEF_ALIGNMENT);
        xController.setTolerance(X_TOLERANCE_REEF_ALIGNMENT);

        yController.setSetpoint(!isRightScore ? Y_SETPOINT_REEF_ALIGNMENT : Y_RIGHT_SETPOINT_REEF_ALIGNMENT);
        yController.setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);

        var latestObservation = visionSubsystem.getLatestTargetFromCamera(0);

        // Get the tag ID and height if it exists
        if (latestObservation.isPresent()) {
            tagID = latestObservation.get().getBestTarget().fiducialId;
            tagHeightMeters = VisionConstants.aprilTagLayout.getTagPose(tagID).get().getTranslation().getZ();
        } else {
            tagID = -1;
        }
    }

    @Override
    public void execute() {
        // If the tag is not found, drive empty
        var latestObservation = visionSubsystem.getLatestTargetFromCamera(0);

        if (!latestObservation.isPresent() || latestObservation.get().getBestTarget().fiducialId != tagID) {
            swerveSubsystem.drive(new Translation2d(), 0, false);
            return;
        }

        this.dontSeeTagTimer.reset();

        // Calculate tag position
        var latestObservationTarget = latestObservation.get().getBestTarget();

        double targetYaw = latestObservationTarget.getYaw();
        double targetDistance = PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.TRANSFORM_TO_CAMERA_0.getZ(),
            tagHeightMeters,
            VisionConstants.TRANSFORM_TO_CAMERA_0.getRotation().getZ(),
            Units.degreesToRadians(latestObservationTarget.getPitch())
        );

        Translation2d targetRelativeToCamera = PhotonUtils.estimateCameraToTargetTranslation(
            targetDistance,
            Rotation2d.fromDegrees(targetYaw)
        );

        double xSpeed = xController.calculate(targetRelativeToCamera.getX());
        double ySpeed = yController.calculate(targetRelativeToCamera.getY());
        double rotValue = rotController.calculate(Units.degreesToRadians(targetYaw));

        // Log
        Logger.recordOutput("Align/XSetpoint", xController.getSetpoint());
        Logger.recordOutput("Align/XCurrent", targetRelativeToCamera.getX());
        Logger.recordOutput("Align/XSpeed", xSpeed);

        Logger.recordOutput("Align/YSetpoint", yController.getSetpoint());
        Logger.recordOutput("Align/YCurrent", targetRelativeToCamera.getY());
        Logger.recordOutput("Align/YSpeed", ySpeed);

        Logger.recordOutput("Align/RSetpoint", rotController.getSetpoint());
        Logger.recordOutput("Align/RCurrent", Units.degreesToRadians(targetYaw));
        Logger.recordOutput("Align/RSpeed", rotValue);

        swerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

        if (!rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
            stopTimer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(), 0, false);
    }

    @Override
    public boolean isFinished() {
        // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
        return (this.dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME) || stopTimer.hasElapsed(POSE_VALIDATION_TIME));
    }
}
