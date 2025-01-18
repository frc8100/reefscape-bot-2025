package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

/**
 * The teleop swerve command.
 */
public class TeleopSwerve extends Command {

    /**
     * The swerve subsystem.
     */
    private final Swerve swerveSubsystem;

    /**
     * The translation input (x), as a double from 0-1.
     * Should be a controller/joystick input.
     */
    private DoubleSupplier translationSupplier;

    /**
     * The strafe input (y), as a double from 0-1.
     * Should be a controller/joystick input.
     */
    private DoubleSupplier strafeSupplier;

    /**
     * The rotation input, as a double from 0-1.
     * Should be a controller/joystick input.
     */
    private DoubleSupplier rotationSupplier;

    /**
     * Whether the swervce is robot centric.
     * Default is always `false`.
     */
    private BooleanSupplier robotCentricSup;

    /**
     * Whether to dampen the input.
     */
    private BooleanSupplier dampen;

    /**
     * The speed multiplier, as a double.
     * Default is `1`
     */
    private DoubleSupplier speedDial;

    /**
     * The PID controller to correct using PID.
     */
    private PIDController rotationController;

    /**
     * Creates the TeleopSwerve command.
     * The parameters are members of this class.
     */
    public TeleopSwerve(
            Swerve swerveSubsystem,
            DoubleSupplier translationSupplier,
            DoubleSupplier strafeSupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier robotCentricSup,
            BooleanSupplier dampen,
            DoubleSupplier speedDial) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

        // Configure the PID Controller
        rotationController = new PIDController(0.01, 0, 0);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(3);

        // Set the values from the contructor
        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.robotCentricSup = robotCentricSup;
        this.dampen = dampen;
        this.speedDial = speedDial;
    }

    /**
     * The function that is called to drive.
     */
    @Override
    public void execute() {
        // Get values and deadband
        double translationInput = translationSupplier.getAsDouble();
        double strafeInput = strafeSupplier.getAsDouble();
        double rotationInput = rotationSupplier.getAsDouble();

        /**
         * The constant to muliply each value by
         */
        double dampenAndSpeedConstant = (dampen.getAsBoolean() ? 0.2 : 1) * ((speedDial.getAsDouble() + 1) / 2);

        double translationValue = MathUtil.applyDeadband(translationInput, Constants.stickDeadband)
                * dampenAndSpeedConstant;
        double strafeValue = MathUtil.applyDeadband(strafeInput, Constants.stickDeadband)
                * dampenAndSpeedConstant;
        double rotationValue = MathUtil.applyDeadband(rotationInput, Constants.stickDeadband)
                * dampenAndSpeedConstant;

        // Heading direction state
        switch (States.driveState) {
            case d0:
                // heading lock
                rotationValue = rotationController.calculate(swerveSubsystem.getYaw().getRadians(), Units.degreesToRadians(0));
                break;
            case d90:
                // heading lock
                rotationValue = rotationController.calculate(swerveSubsystem.getYaw().getRadians(), Units.degreesToRadians(90));
                break;
            case d180:
                // heading lock
                rotationValue = rotationController.calculate(swerveSubsystem.getYaw().getRadians(), Units.degreesToRadians(180));
                break;
            case d270:
                // heading lock
                rotationValue = rotationController.calculate(swerveSubsystem.getYaw().getRadians(), Units.degreesToRadians(270));
                break;
            case standard:
                // normal
                rotationValue = rotationValue * SwerveConfig.maxAngularVelocity;
                break;
        }

        // Log the values
        Logger.recordOutput("Swerve/TranslationInput", translationInput);
        Logger.recordOutput("Swerve/StrafeInput", strafeInput);
        Logger.recordOutput("Swerve/RotationInput", rotationInput);
        
        Logger.recordOutput("Swerve/TranslationValue", translationValue);
        Logger.recordOutput("Swerve/StrafeValue", strafeValue);
        Logger.recordOutput("Swerve/RotationValue", rotationValue);

        // Drive
        swerveSubsystem.drive(
                new Translation2d(translationValue, strafeValue).times(SwerveConfig.maxSpeed),
                rotationValue,
                !robotCentricSup.getAsBoolean(),
                true);
    }
}
