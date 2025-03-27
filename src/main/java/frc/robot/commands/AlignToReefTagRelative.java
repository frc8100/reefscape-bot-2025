// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.lib.util.TunableValue;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDrive;
import org.littletonrobotics.junction.Logger;

public class AlignToReefTagRelative extends Command {

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
        "Align/RotationSetpoint",
        ROT_SETPOINT_REEF_ALIGNMENT,
        (double value) -> ROT_SETPOINT_REEF_ALIGNMENT = value
    );
    private static final TunableValue X_SETPOINT_REEF_ALIGNMENT_TUNABLE = new TunableValue(
        "Align/XSetpoint",
        X_SETPOINT_REEF_ALIGNMENT,
        (double value) -> X_SETPOINT_REEF_ALIGNMENT = value
    );
    private static final TunableValue Y_SETPOINT_REEF_ALIGNMENT_TUNABLE = new TunableValue(
        "Align/YSetpoint",
        Y_SETPOINT_REEF_ALIGNMENT,
        (double value) -> Y_SETPOINT_REEF_ALIGNMENT = value
    );

    static {
        new TunableValue("Align/Right/RotationSetpoint", ROT_RIGHT_SETPOINT_REEF_ALIGNMENT, (double value) ->
            ROT_RIGHT_SETPOINT_REEF_ALIGNMENT = value
        );
        new TunableValue("Align/Right/XSetpoint", X_RIGHT_SETPOINT_REEF_ALIGNMENT, (double value) ->
            X_RIGHT_SETPOINT_REEF_ALIGNMENT = value
        );
        new TunableValue("Align/Right/YSetpoint", Y_RIGHT_SETPOINT_REEF_ALIGNMENT, (double value) ->
            Y_RIGHT_SETPOINT_REEF_ALIGNMENT = value
        );
    }

    private static double X_P = 0.85;
    private static double X_D = 0.005;

    private static double Y_P = 0.75;
    private static double Y_D = 0.005;

    private static double R_P = 0.04;
    private static double R_D = 0.002;

    private static final TunableValue X_P_TUNABLE = new TunableValue("Align/XP", X_P, (double value) -> X_P = value);
    private static final TunableValue X_D_TUNABLE = new TunableValue("Align/XD", X_D, (double value) -> X_D = value);

    private static final TunableValue Y_P_TUNABLE = new TunableValue("Align/YP", Y_P, (double value) -> Y_P = value);
    private static final TunableValue Y_D_TUNABLE = new TunableValue("Align/YD", Y_D, (double value) -> Y_D = value);

    private static final TunableValue R_P_TUNABLE = new TunableValue("Align/RP", R_P, (double value) -> R_P = value);
    private static final TunableValue R_D_TUNABLE = new TunableValue("Align/RD", R_D, (double value) -> R_D = value);

    private static final double DONT_SEE_TAG_WAIT_TIME = 1;
    private static final double POSE_VALIDATION_TIME = 0.3;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;

    /**
     * @return Whether or not the limelight sees the tag and can align.
     */
    public static boolean getCanAlignToReef() {
        return LimelightHelpers.getTV("");
    }

    private boolean isRightScore;
    private Timer dontSeeTagTimer;
    private Timer stopTimer;
    private SwerveDrive swerveSubsystem;
    private double tagID = -1;

    public AlignToReefTagRelative(boolean isRightScore, SwerveDrive swerveSubsystem) {
        this.isRightScore = isRightScore;
        this.swerveSubsystem = swerveSubsystem;
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

        tagID = LimelightHelpers.getFiducialID("");
    }

    @Override
    public void execute() {
        // If the tag is not found, drive empty
        if (!(LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID)) {
            swerveSubsystem.drive(new Translation2d(), 0, false);
            return;
        }

        this.dontSeeTagTimer.reset();

        double[] positions = LimelightHelpers.getBotPose_TargetSpace("");

        // Logger.recordOutput("Align/Positions", positions);

        Logger.recordOutput("Align/XSetpoint", xController.getSetpoint());
        Logger.recordOutput("Align/XCurrent", positions[2]);

        Logger.recordOutput("Align/YSetpoint", yController.getSetpoint());
        Logger.recordOutput("Align/YCurrent", positions[0]);

        Logger.recordOutput("Align/RSetpoint", rotController.getSetpoint());
        Logger.recordOutput("Align/RCurrent", positions[4]);

        double xSpeed = xController.calculate(positions[2]);
        double ySpeed = -yController.calculate(positions[0]);
        double rotValue = -rotController.calculate(positions[4]);

        Logger.recordOutput("Align/Speeds", new double[] { xSpeed, ySpeed, rotValue });

        swerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

        // driveController.calculateRobotRelativeSpeeds(driveSubsystem.getPose(), new PathPlannerTrajectoryState())

        if (!rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
            stopTimer.reset();
        }
        // SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
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
