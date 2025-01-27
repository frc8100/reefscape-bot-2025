package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    private final Swerve swerveSubsystem = new Swerve();

    // private final PoseEstimator s_PoseEstimator = new PoseEstimator();

    // Dashboard inputs
    /**
     * Chooses the auto command
     */
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new TeleopSwerve(
                swerveSubsystem,
                Controls.Drive::getTranslationAxis,
                Controls.Drive::getStrafeAxis,
                Controls.Drive::getRotationAxis,
                Controls.Drive::isRobotCentric,
                Controls.Drive::isDampen,
                Controls.Drive::getSpeedMultiplier));

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        // autoChooser.addOption(
        //         "Drive Wheel Radius Characterization",
        // DriveCommands.wheelRadiusCharacterization(drive));
        // autoChooser.addOption(
        //         "Drive Simple FF Characterization",
        // DriveCommands.feedforwardCharacterization(drive));
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Forward)",
        //         drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Reverse)",
        //         drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        //         "Drive SysId (Dynamic Forward)",
        // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Drive SysId (Dynamic Reverse)",
        // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        Controls.Drive.zeroGyro.onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));

        // Heading lock bindings
        Controls.Drive.up
                .onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d90))
                .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));
        Controls.Drive.left
                .onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d180))
                .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));
        Controls.Drive.right
                .onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d0))
                .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));
        Controls.Drive.down
                .onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d270))
                .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));

        // Up system bindings
        // TODO: Switch up system to event commands
        // Note: on the Logitech controller, x and a are swapped
        // Controls.upController.b().whileTrue(m_intake.intake());
        // Controls.upController.x().whileTrue(m_intake.eintake());
        // Controls.upController.rightBumper().whileTrue(m_arm.up());
        // Controls.upController.leftBumper().whileTrue(m_arm.down());

        // Controls.upController.leftTrigger(0.75).whileTrue(m_launcher.pushOut());
        // Controls.upController.rightTrigger(0.75).whileTrue(m_launcher.pushIn());
        // Controls.upController.axisGreaterThan(Controls.Up.launcherInAxis).ifHigh(m_launcher.pushIn());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
        // return null;
    }
}
