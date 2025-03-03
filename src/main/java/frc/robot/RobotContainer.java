package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SwerveSysidRoutines;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.claw.ClawConstants;
import frc.robot.subsystems.superstructure.claw.ClawIOSim;
import frc.robot.subsystems.superstructure.claw.ClawIOSpark;
import frc.robot.subsystems.superstructure.claw.ClawSim;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
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
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeReefSimulation;
import org.littletonrobotics.junction.Logger;
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
    private final Elevator elevatorSubsystem;

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
            case REAL:
                // Real robot, instantiate hardware IO implementations
                swerveSubsystem = new Swerve(
                    new GyroIOPigeon2(),
                    new ModuleIO[] {
                        new ModuleIOSpark(0, SwerveConstants.Swerve.Mod0.constants),
                        new ModuleIOSpark(1, SwerveConstants.Swerve.Mod1.constants),
                        new ModuleIOSpark(2, SwerveConstants.Swerve.Mod2.constants),
                        new ModuleIOSpark(3, SwerveConstants.Swerve.Mod3.constants),
                    }
                );

                visionSubsystem = new Vision(
                    swerveSubsystem::addVisionMeasurement,
                    new VisionIOLimelight(VisionConstants.CAMERA_0_NAME, swerveSubsystem::getRotation)
                );

                clawSubsystem = new Claw(new ClawIOSpark());
                break;
            default:
            case SIM:
                // Create a reefscape arena
                var arenaSimulation = new Arena2025Reefscape();

                // Set up the simulated arena by overriding the instance and adding the pieces
                SimulatedArena.overrideInstance(arenaSimulation);
                SimulatedArena.getInstance().addCustomSimulation(new ReefscapeReefSimulation(arenaSimulation));
                SimulatedArena.getInstance().placeGamePiecesOnField();

                // Create a simulated drive
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
                    driveSimulation
                );

                // Create a simulated vision subsystem
                visionSubsystem = new Vision(
                    swerveSubsystem::addVisionMeasurement,
                    new VisionIOPhotonSim(
                        VisionConstants.CAMERA_0_NAME,
                        VisionConstants.TRANSFORM_TO_CAMERA_0,
                        swerveSubsystem::getActualPose
                    )
                );

                // Create a simulated claw
                clawSubsystem = new ClawSim();

                // TODO: Add behavior chooser
                // Create an opponent robot simulation
                // OpponentRobotSim opponentRobotSim1 =
                //         new OpponentRobotSim(new Pose2d(10, 2, new Rotation2d()),
                // OpponentRobotBehavior.TeleopSwerve);

                // Create another joystick drive for the opponent robot
                // Controls.Drive opponentRobotDriveControls = new Controls.JoystickDrive(new Joystick(1));

                // Set the default command for the opponent robot
                // opponentRobotSim1.setDefaultCommand(
                //         new TeleopSwerve(opponentRobotSim1, opponentRobotDriveControls, false));
                // opponentRobotSim1.setDefaultCommand(opponentRobotSim1.opponentRobotPathfindToPoseSupplier(swerveSubsystem::getActualPose));

                // OpponentRobotSim opponentRobotSim2 = new OpponentRobotSim(new Pose2d(13, 6, new Rotation2d()));
                // opponentRobotSim2.setDefaultCommand(opponentRobotSim2.opponentRobotPathfindToPoseSupplier(swerveSubsystem::getActualPose));

                // TODO: refactor
                // opponentRobotDriveControls
                //         .getJoystickButtonOf(opponentRobotDriveControls.zeroGyroButton)
                //         .onTrue(new InstantCommand(() -> opponentRobotSim1.zeroGyro()));
                break;
        }

        // TODO: Real elevator IO implementation
        elevatorSubsystem = new Elevator(new ElevatorIOSim());

        // TODO: add switch for controller and joystick
        swerveSubsystem.setDefaultCommand(new TeleopSwerve(swerveSubsystem, Controls.mainDriveControls, true));
        // swerveSubsystem.setDefaultCommand(new TeleopSwerve(swerveSubsystem, new
        // Controls.JoystickDrive(Controls.mainDriverController)));

        // Set up auto routines
        autoRoutines = new AutoRoutines(swerveSubsystem);

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            SwerveSysidRoutines.wheelRadiusCharacterization(swerveSubsystem)
        );
        autoChooser.addOption(
            "Drive Simple FF Characterization",
            SwerveSysidRoutines.feedforwardCharacterization(swerveSubsystem)
        );
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)",
            swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)",
            swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );

        // TODO: Temporary
        autoChooser.addOption("Coral and Go To All Reefs Test", autoRoutines.getCoralAndGoToAllReefsTest());

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
        Controls.mainDriveControls
            .getJoystickButtonOf(Controls.mainDriveControls.zeroGyroButton)
            .onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        // TODO: Heading lock?

        // Claw
        clawSubsystem.setDefaultCommand(clawSubsystem.getRunCommand(Controls.Claw::getIntakeOrOuttake));

        Controls.Claw.moveToL1.onTrue(clawSubsystem.getAngleCommand(ClawConstants.RotationPositions.L1ANGLE));
        Controls.Claw.moveToL2.onTrue(clawSubsystem.getAngleCommand(ClawConstants.RotationPositions.L2ANGLE));
        Controls.Claw.moveToL3.onTrue(clawSubsystem.getAngleCommand(ClawConstants.RotationPositions.L3ANGLE));
        Controls.Claw.moveToL4.onTrue(clawSubsystem.getAngleCommand(ClawConstants.RotationPositions.L4ANGLE));

        Controls.Claw.resetClawButton.onTrue(
            clawSubsystem.getAngleCommand(ClawConstants.RotationPositions.RESTING_ANGLE)
        );

        // Elevator
        Controls.Elevator.moveToL1.onTrue(elevatorSubsystem.getPositionCommand(ElevatorConstants.Position.L1_DISTANCE));
        Controls.Elevator.moveToL2.onTrue(elevatorSubsystem.getPositionCommand(ElevatorConstants.Position.L2_DISTANCE));
        Controls.Elevator.moveToL3.onTrue(elevatorSubsystem.getPositionCommand(ElevatorConstants.Position.L3_DISTANCE));
        Controls.Elevator.moveToL4.onTrue(elevatorSubsystem.getPositionCommand(ElevatorConstants.Position.L4_DISTANCE));
        // Test
        // Controls.mainDriveControls.goToCoralStation1.whileTrue(autoRoutines.getCoralFromStation(1));
        // Controls.mainDriveControls.goToReef1.whileTrue(autoRoutines.goToReef(1));
        // Controls.mainDriveControls
        //         .getJoystickButtonOf(Controls.mainDriveControls.goToCoralStation1Button)
        //         .whileTrue(autoRoutines.getCoralFromStation(1));
        // Controls.mainDriveControls
        //         .getJoystickButtonOf(Controls.mainDriveControls.goToReef1Button)
        //         .whileTrue(autoRoutines.goToReef(1));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /**
     * Run in `Robot.simulationPeriodic()` to update the subsystem-specific simulation.
     * @throws IllegalStateException if the subsystems are not simulated
     */
    public void simulationPeriodic() {
        // Check all the subsystems are simulated
        if (!(swerveSubsystem instanceof SwerveSim && clawSubsystem instanceof ClawSim)) {
            throw new IllegalStateException("Subsystems are not simulated");
        }

        // Update the simulation
        ((ClawSim) clawSubsystem).simulationPeriodic((SwerveSim) swerveSubsystem, elevatorSubsystem);
    }

    /**
     * Run in `Robot.periodic()`.
     */
    public void periodic() {
        // Update telemetry for claw position
        Logger.recordOutput("ComponentPositions/Claw", clawSubsystem.getPose(elevatorSubsystem.getStage2Pose()));
    }
}
