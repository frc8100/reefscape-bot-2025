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
import frc.lib.util.TunableValue;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AlignToReefTagRelative extends Command {

    public static double ROT_SETPOINT_REEF_ALIGNMENT = 0; // Rotation
    public static double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
    public static double X_SETPOINT_REEF_ALIGNMENT = -0.05; // Vertical pose
    public static double X_TOLERANCE_REEF_ALIGNMENT = 0.04;
    public static double Y_SETPOINT_REEF_ALIGNMENT = 0.16; // Horizontal pose
    public static double Y_TOLERANCE_REEF_ALIGNMENT = 0.04;

    public static final TunableValue ROT_SETPOINT_REEF_ALIGNMENT_TUNABLE = new TunableValue(
        "Align/RotationSetpoint",
        ROT_SETPOINT_REEF_ALIGNMENT,
        (double value) -> ROT_SETPOINT_REEF_ALIGNMENT = value
    );

    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 0.3;

    private final PIDController xController = new PIDController(1, 0.0, 0.02);
    private final PIDController yController = new PIDController(1, 0.0, 0.02);
    private final PIDController rotController = new PIDController(1, 0.0, 0.05);
    // private final ProfiledPIDController rotController = new ProfiledPIDController(
    //     1,
    //     0,
    //     0,
    //     new TrapezoidProfile.Constraints(
    //         SwerveConfig.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
    //         SwerveConfig.MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond)
    //     )
    // );

    // private final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
    //     new PIDConstants(1.0, 0.02),
    //     new PIDConstants(1.0, 0.05)
    // );

    // private final HolonomicDriveController driveController = new HolonomicDriveController(
    //     xController,
    //     yController,
    //     rotController
    // );

    private boolean isRightScore;
    private Timer dontSeeTagTimer, stopTimer;
    private SwerveDrive driveSubsystem;
    private double tagID = -1;

    public AlignToReefTagRelative(boolean isRightScore, SwerveDrive driveSubsystem) {
        this.isRightScore = isRightScore;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
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

        rotController.setSetpoint(ROT_SETPOINT_REEF_ALIGNMENT);
        rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);

        xController.setSetpoint(X_SETPOINT_REEF_ALIGNMENT);
        xController.setTolerance(X_TOLERANCE_REEF_ALIGNMENT);

        yController.setSetpoint(isRightScore ? Y_SETPOINT_REEF_ALIGNMENT : -Y_SETPOINT_REEF_ALIGNMENT);
        yController.setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);

        tagID = LimelightHelpers.getFiducialID("");
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
            this.dontSeeTagTimer.reset();

            double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
            SmartDashboard.putNumber("x", postions[2]);

            double xSpeed = xController.calculate(postions[2]);
            SmartDashboard.putNumber("xspeed", xSpeed);
            double ySpeed = -yController.calculate(postions[0]);
            double rotValue = -rotController.calculate(postions[4]);

            driveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

            // driveController.calculateRobotRelativeSpeeds(driveSubsystem.getPose(), new PathPlannerTrajectoryState())

            if (!rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
                stopTimer.reset();
            }
        } else {
            driveSubsystem.drive(new Translation2d(), 0, false);
        }

        SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(new Translation2d(), 0, false);
    }

    @Override
    public boolean isFinished() {
        // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
        return (this.dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME) || stopTimer.hasElapsed(POSE_VALIDATION_TIME));
    }
}
