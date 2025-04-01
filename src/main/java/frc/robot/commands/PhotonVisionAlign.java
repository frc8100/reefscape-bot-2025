package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.lib.util.TunableValue;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.path.AutoRoutines;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

/**
 * Aligns the robot to a target using PhotonVision and PID control.
 */
public class PhotonVisionAlign extends Command {

    /**
     * A supplier that provides the target robot rotation for a given tag ID.
     */
    @FunctionalInterface
    public static interface TargetRobotRotationSupplier {
        public Rotation2d getTargetRobotRotation(int tagID);
    }

    private static final String DASHBOARD_STRING = "AlignPV/";

    private static Angle ROT_SETPOINT_REEF_ALIGNMENT = Radians.of(0);
    private static Angle ROT_RIGHT_SETPOINT_REEF_ALIGNMENT = Radians.of(0);
    private static double ROT_TOLERANCE_REEF_ALIGNMENT = 1;

    private static Distance X_SETPOINT_REEF_ALIGNMENT = Meters.of(0.3); // TZ / Vertical pose -0.16
    private static Distance X_RIGHT_SETPOINT_REEF_ALIGNMENT = Meters.of(0.3);
    private static double X_TOLERANCE_REEF_ALIGNMENT = 0.04;

    private static Distance Y_SETPOINT_REEF_ALIGNMENT = Meters.of(0); // Horizontal pose
    private static Distance Y_RIGHT_SETPOINT_REEF_ALIGNMENT = Meters.of(-0.15);
    private static double Y_TOLERANCE_REEF_ALIGNMENT = 0.04;

    private static final TunableValue ROT_SETPOINT_REEF_ALIGNMENT_TUNABLE = new TunableValue(
        DASHBOARD_STRING + "RotationSetpoint",
        ROT_SETPOINT_REEF_ALIGNMENT.in(Radians),
        (double value) -> ROT_SETPOINT_REEF_ALIGNMENT = Radians.of(value)
    );
    private static final TunableValue X_SETPOINT_REEF_ALIGNMENT_TUNABLE = new TunableValue(
        DASHBOARD_STRING + "XSetpoint",
        X_SETPOINT_REEF_ALIGNMENT.in(Meters),
        (double value) -> X_SETPOINT_REEF_ALIGNMENT = Meters.of(value)
    );
    private static final TunableValue Y_SETPOINT_REEF_ALIGNMENT_TUNABLE = new TunableValue(
        DASHBOARD_STRING + "YSetpoint",
        Y_SETPOINT_REEF_ALIGNMENT.in(Meters),
        (double value) -> Y_SETPOINT_REEF_ALIGNMENT = Meters.of(value)
    );

    private static double X_P = 3;
    private static double X_D = 0.05;

    private static double Y_P = 3;
    private static double Y_D = 0.05;

    private static double R_P = 5;
    private static double R_D = 0.05;

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

    private final TargetRobotRotationSupplier targetRobotRotationSupplier;
    private final Supplier<Rotation2d> robotRotationSupplier;

    private SwerveDrive swerveSubsystem;
    private Vision visionSubsystem;

    private boolean isRightScore;
    private Timer dontSeeTagTimer;
    private Timer stopTimer;

    private int tagID = -1;
    private double tagHeightMeters = 0.0;

    public PhotonVisionAlign(
        boolean isRightScore,
        // TargetRobotRotationSupplier targetRobotRotationSupplier,
        SwerveDrive swerveSubsystem,
        // Supplier<Rotation2d> robotRotationSupplier,
        Vision visionSubsystem
    ) {
        this.isRightScore = isRightScore;
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;
        // this.targetRobotRotationSupplier = targetRobotRotationSupplier;
        this.targetRobotRotationSupplier = AutoRoutines::getReefRotationOfTargetPoseBasedOnTagId;
        this.robotRotationSupplier = swerveSubsystem::getRotation;
        addRequirements(swerveSubsystem);

        xController = new PIDController(X_P_TUNABLE.get(), 0.0, X_D_TUNABLE.get());
        yController = new PIDController(Y_P_TUNABLE.get(), 0.0, Y_D_TUNABLE.get());
        rotController = new PIDController(R_P_TUNABLE.get(), 0.0, R_D_TUNABLE.get());

        rotController.enableContinuousInput(-Math.PI, Math.PI);

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
        rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);

        xController.setSetpoint(
            !isRightScore ? X_SETPOINT_REEF_ALIGNMENT.in(Meters) : X_RIGHT_SETPOINT_REEF_ALIGNMENT.in(Meters)
        );
        xController.setTolerance(X_TOLERANCE_REEF_ALIGNMENT);

        yController.setSetpoint(
            !isRightScore ? Y_SETPOINT_REEF_ALIGNMENT.in(Meters) : Y_RIGHT_SETPOINT_REEF_ALIGNMENT.in(Meters)
        );
        yController.setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);

        tagID = -1;
    }

    @Override
    public void execute() {
        // If the tag is not found, drive empty
        var latestObservation = visionSubsystem.getLatestTargetFromCamera(0);

        // Set the tag ID and height if it exists and is not already set
        if (tagID == -1 && latestObservation.isPresent()) {
            tagID = latestObservation.get().getBestTarget().fiducialId;

            // If the tag ID is not a reef tag, try again
            if (!VisionConstants.REEF_APRILTAG_IDS.contains(tagID)) {
                // Reset tagID to try again on the next loop
                tagID = -1;
                return;
            }

            // Logger.recordOutput("Align/TagFound", true);
            tagHeightMeters = VisionConstants.aprilTagLayout.getTagPose(tagID).get().getTranslation().getZ();

            rotController.setSetpoint(targetRobotRotationSupplier.getTargetRobotRotation(tagID).getRadians());
        }
        Logger.recordOutput("Align/TagId", tagID);

        // If the tag is not found, stop the robot
        if (
            !latestObservation.isPresent() ||
            !latestObservation.get().hasTargets() ||
            latestObservation.get().getBestTarget().fiducialId != tagID
        ) {
            // Logger.recordOutput("Align/TagFound", false);
            // Logger.recordOutput(
            //     "Align/CurrentBestTag",
            //     latestObservation.isPresent() ? latestObservation.get().getBestTarget().fiducialId : -1
            // );

            swerveSubsystem.drive(new Translation2d(), 0, false);
            return;
        }

        var latestObservationTarget = latestObservation.get().getBestTarget();

        this.dontSeeTagTimer.reset();
        Logger.recordOutput("Align/TagFound", true);

        // Calculate tag position
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

        double xSpeed = -xController.calculate(targetRelativeToCamera.getX());
        double ySpeed = yController.calculate(targetRelativeToCamera.getY());
        double rotValue = rotController.calculate(robotRotationSupplier.get().getRadians());

        // Log
        Logger.recordOutput("Align/XSetpoint", xController.getSetpoint());
        Logger.recordOutput("Align/XCurrent", targetRelativeToCamera.getX());
        Logger.recordOutput("Align/XSpeed", xSpeed);

        Logger.recordOutput("Align/YSetpoint", yController.getSetpoint());
        Logger.recordOutput("Align/YCurrent", targetRelativeToCamera.getY());
        Logger.recordOutput("Align/YSpeed", ySpeed);

        Logger.recordOutput("Align/RSetpoint", rotController.getSetpoint());
        Logger.recordOutput("Align/RCurrent", robotRotationSupplier.get().getRadians());
        Logger.recordOutput("Align/RSpeed", rotValue);

        swerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

        // Robot is at setpoint, reset the timers to ensure we hold position
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
        return (dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME) || stopTimer.hasElapsed(POSE_VALIDATION_TIME));
    }
}
