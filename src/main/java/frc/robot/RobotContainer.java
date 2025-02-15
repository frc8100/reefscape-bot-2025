package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.claw.ClawIOSpark;
import frc.robot.subsystems.swerve.OpponentRobotSim;
import frc.robot.subsystems.swerve.OpponentRobotSim.OpponentRobotBehavior;
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
    private final Claw clawSubsystem;

    private AutoRoutines autoRoutines;

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

                // Create an opponent robot simulation
                OpponentRobotSim opponentRobotSim1 =
                        new OpponentRobotSim(new Pose2d(10, 2, new Rotation2d()), OpponentRobotBehavior.TeleopSwerve);

                // Create another joystick drive for the opponent robot
                Controls.Drive opponentRobotDriveControls = new Controls.JoystickDrive(new Joystick(1));

                // Set the default command for the opponent robot
                opponentRobotSim1.setDefaultCommand(
                        new TeleopSwerve(opponentRobotSim1, opponentRobotDriveControls, false));
                // opponentRobotSim1.setDefaultCommand(opponentRobotSim1.opponentRobotPathfindToPoseSupplier(swerveSubsystem::getActualPose));

                // OpponentRobotSim opponentRobotSim2 = new OpponentRobotSim(new Pose2d(13, 6, new Rotation2d()));
                // opponentRobotSim2.setDefaultCommand(opponentRobotSim2.opponentRobotPathfindToPoseSupplier(swerveSubsystem::getActualPose));

                // TODO: refactor
                opponentRobotDriveControls
                        .getJoystickButtonOf(opponentRobotDriveControls.zeroGyroButton)
                        .onTrue(new InstantCommand(() -> opponentRobotSim1.zeroGyro()));
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

        // TODO: sim implementation
        clawSubsystem = new Claw(new ClawIOSpark());

        // TODO: add switch for controller and joystick
        swerveSubsystem.setDefaultCommand(new TeleopSwerve(swerveSubsystem, Controls.mainDriveControls, true));
        // swerveSubsystem.setDefaultCommand(new TeleopSwerve(swerveSubsystem, new
        // Controls.JoystickDrive(Controls.mainDriverController)));

        // Set up auto routines
        autoRoutines = new AutoRoutines(swerveSubsystem);

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
        // Controls.mainDriveControls.zeroGyro.onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
        Controls.mainDriveControls
                .getJoystickButtonOf(Controls.mainDriveControls.zeroGyroButton)
                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));

        // Heading lock bindings
        // TODO: Reimplement
        // Controls.mainDriveControls.up
        //         .onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d90))
        //         .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));
        // Controls.mainDriveControls.left
        //         .onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d180))
        //         .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));
        // Controls.mainDriveControls.right
        //         .onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d0))
        //         .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));
        // Controls.mainDriveControls.down
        //         .onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d270))
        //         .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));

        // Claw
        // clawSubsystem.setDefaultCommand(
        //         Commands.run(() -> clawSubsystem.runClaw(Controls.Arm.getIntakeOrOuttake()), clawSubsystem));
        clawSubsystem.setDefaultCommand(clawSubsystem.getRunCommand(Controls.Arm::getIntakeOrOuttake));

        // Test

        // Controls.mainDriveControls.goToCoralStation1.whileTrue(autoRoutines.getCoralFromStation(1));
        // Controls.mainDriveControls.goToReef1.whileTrue(autoRoutines.goToReef(1));
        Controls.mainDriveControls
                .getJoystickButtonOf(Controls.mainDriveControls.goToCoralStation1Button)
                .whileTrue(autoRoutines.getCoralFromStation(1));
        Controls.mainDriveControls
                .getJoystickButtonOf(Controls.mainDriveControls.goToReef1Button)
                .whileTrue(autoRoutines.goToReef(1));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return autoChooser.get();
        // TODO: Temporary
        return autoRoutines.getCoralAndGoToAllReefsTest();
    }
}
