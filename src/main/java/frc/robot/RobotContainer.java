package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveSim;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOSpark;
import frc.robot.subsystems.swerve.path.AutoRoutines;
import frc.robot.subsystems.swerve.path.SwervePathFollow;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    private final Vision visionSubsystem;
    private final SwerveDrive swerveSubsystem;
    private final ArmSubsystem armSubsystem = new ArmSubsystem();

    /**
     * The simulation of the robot's drive. Set to null if not in simulation mode.
     */
    private SwerveDriveSimulation driveSimulation = null;

    // Dashboard inputs
    /**
     * Chooses the auto command
     */
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            default:
            case REAL:
                // Real robot, instantiate hardware IO implementations
                swerveSubsystem = new Swerve(new GyroIOPigeon2(), new ModuleIO[] {
                    new ModuleIOSpark(0, SwerveConstants.Swerve.Mod0.constants),
                    new ModuleIOSpark(1, SwerveConstants.Swerve.Mod1.constants),
                    new ModuleIOSpark(2, SwerveConstants.Swerve.Mod2.constants),
                    new ModuleIOSpark(3, SwerveConstants.Swerve.Mod3.constants)
                });

                visionSubsystem = new Vision(
                        swerveSubsystem::addVisionMeasurement,
                        new VisionIOLimelight(VisionConstants.camera0Name, swerveSubsystem::getRotation));
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                driveSimulation = new SwerveDriveSimulation(SwerveConfig.mapleSimConfig, SwerveConfig.initialPose);

                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                SwerveModuleSimulation[] moduleSims = driveSimulation.getModules();

                swerveSubsystem = new SwerveSim(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIO[] {
                            new ModuleIOSim(moduleSims[0]),
                            new ModuleIOSim(moduleSims[1]),
                            new ModuleIOSim(moduleSims[2]),
                            new ModuleIOSim(moduleSims[3]),
                        },
                        driveSimulation);

                // swerveSubsystem = new SwerveSim();

                visionSubsystem = new Vision(
                        swerveSubsystem::addVisionMeasurement,
                        new VisionIOPhotonSim(
                                VisionConstants.camera0Name,
                                VisionConstants.robotToCamera0,
                                swerveSubsystem::getActualPose));
                break;

                // default:
                //     // Replayed robot, disable IO implementations
                //     // swerveSubsystem = new Swerve(
                //     //         new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new
                // ModuleIO() {});
                //     swerveSubsystem = new Swerve(new GyroIO() {}, new ModuleIO[] {
                //         new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {},
                //     });
                //     break;
        }

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

        // Arm
        armSubsystem.setDefaultCommand(
                Commands.run(() -> armSubsystem.runClaw(Controls.Arm.getIntakeOrOuttake()), armSubsystem));

        // Test
        // Controls.Drive.testFollowPose.whileTrue(swerveSubsystem.goToPoseCommand(new Pose2d(2, 2, new Rotation2d())));

        // Controls.Drive.testFollowPose.whileTrue(
        //         // Create a new command that creates a command to follow a pose
        //         Commands.runOnce(
        //                 () -> swerveSubsystem
        //                         .goToPoseCommand(new Pose2d(2, 2, new Rotation2d()))
        //                         .andThen(Commands.waitSeconds(10.0))
        //                         .until(() -> !Controls.Drive.testFollowPose.getAsBoolean())
        //                         .schedule(),
        //                 swerveSubsystem));

        // TODO: refactor
        SwervePathFollow pathFollow = swerveSubsystem.getPathfindingInstance();

        // Controls.Drive.testFollowPose.whileTrue(
        //     swerveSubsystem
        //         .getPathfindingInstance()
        //         .generateDynamicPathFollow(swerveSubsystem, SwervePathFollow.FieldLocations.coralStation1));

        // Controls.Drive.testFollowPose.whileTrue(pathFollow
        //         .generateDynamicPathFollow(swerveSubsystem, SwervePathFollow.FieldLocations.coralStation1)
        //         .andThen(Commands.waitSeconds(2.0))
        //         .andThen(pathFollow.generateDynamicPathFollow(swerveSubsystem,
        // SwervePathFollow.FieldLocations.reef1))
        //         .andThen(Commands.waitSeconds(2.0)));

        // Controls.Drive.testFollowPose.whileTrue(
        //         pathFollow.generateDynamicCommands((BooleanSupplier shouldBeRunning) -> pathFollow
        //                 .generateGoToPoseCommand(SwervePathFollow.FieldLocations.coralStation1)
        //                 .andThen(pathFollow.generateGoToPoseCommand(SwervePathFollow.FieldLocations.reef1))
        //                 .onlyWhile(shouldBeRunning)));

        // Controls.Drive.testFollowPose.whileTrue(pathFollow
        //         .generateGoToPoseCommand(SwervePathFollow.FieldLocations.coralStation1)
        //         .andThen(pathFollow.generateGoToPoseCommand(SwervePathFollow.FieldLocations.reef1)));

        AutoRoutines autoRoutines = new AutoRoutines(swerveSubsystem);

        Controls.Drive.testFollowPose.whileTrue(autoRoutines.getCoralFromStation1());

        // swerveSubsystem.registerRoutineWhileCondition(
        //         new SwerveDrive.RoutineWithCondition(() -> Controls.Drive.testFollowPose.getAsBoolean(), () -> {
        //             return autoRoutines.getCoralFromStation1();
        //         }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
