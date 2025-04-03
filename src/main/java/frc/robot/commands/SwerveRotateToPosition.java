package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SwerveRotateToPosition extends Command {

    private double R_P = 3;
    private double R_D = 0.05;

    private PIDController pidController = new PIDController(R_D, 0, R_D);

    private Supplier<Rotation2d> rotationSupplier;

    private SwerveDrive swerveSubsystem;

    public SwerveRotateToPosition(Supplier<Rotation2d> rotationSupplier, SwerveDrive swerveSubsystem) {
        this.rotationSupplier = rotationSupplier;
        this.swerveSubsystem = swerveSubsystem;

        pidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        double rotValue = pidController.calculate(rotationSupplier.get().getRadians());

        Logger.recordOutput("Align/RSetpoint", pidController.getSetpoint());
        Logger.recordOutput("Align/RCurrent", rotationSupplier.get().getRadians());
        Logger.recordOutput("Align/RSpeed", rotValue);

        swerveSubsystem.drive(new Translation2d(0, 0), rotValue, false);
    }
}
