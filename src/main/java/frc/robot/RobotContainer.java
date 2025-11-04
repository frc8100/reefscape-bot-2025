package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.LimelightHelpers;
import frc.lib.util.EmptySimulationArena;
import frc.lib.util.TunableValue;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.SwerveSysidRoutines;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.questnav.QuestNavIO;
import frc.robot.subsystems.questnav.QuestNavIOReal;
import frc.robot.subsystems.questnav.QuestNavIOSim;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.claw.ClawConstants;
import frc.robot.subsystems.superstructure.claw.ClawIOSpark;
import frc.robot.subsystems.superstructure.claw.ClawSim;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSpark;
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
import frc.robot.subsystems.swerve.path.AutoRoutines.FieldLocations;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonSim;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionSim;
import frc.robot.subsystems.vision.VisionSim.NeuralDetectorSimPipeline;
import java.util.Set;
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
    private final QuestNavSubsystem questNavSubsystem;
    private final Swerve swerveSubsystem;
    private final Claw clawSubsystem;
    private final Elevator elevatorSubsystem;

    private final AutoRoutines autoRoutines;

    /**
     * The simulation of the robot's drive. Set to null if not in simulation mode.
     */
    private SwerveDriveSimulation driveSimulation = null;

    // Dashboard inputs
    /**
     * Chooses the auto command
     */
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
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
                    // new VisionIOLimelight(VisionConstants.CAMERA_0_NAME, swerveSubsystem::getRotation)
                    // new VisionIOPhotonVision(VisionConstants.CAMERA_0_NAME, VisionConstants.TRANSFORM_TO_CAMERA_0)
                    new VisionIO() {}
                );

                questNavSubsystem = new QuestNavSubsystem(swerveSubsystem::addVisionMeasurement, new QuestNavIOReal());

                clawSubsystem = new Claw(new ClawIOSpark());
                elevatorSubsystem = new Elevator(new ElevatorIOSpark());
                break;
            default:
            case SIM:
                // Override the arena
                if (Constants.disableSimArena) {
                    // Use an empty arena for SysId to reduce obstacles
                    SimulatedArena.overrideInstance(new EmptySimulationArena());
                } else {
                    // Use the reefscape arena for normal sim
                    SimulatedArena.overrideInstance(new Arena2025Reefscape());
                }

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
                NeuralDetectorSimPipeline[] simPipelines = VisionSim.getDetectorPipelines(
                    SimulatedArena.getInstance()::getGamePiecesPosesByType
                );

                visionSubsystem = new VisionSim(
                    swerveSubsystem::addVisionMeasurement,
                    swerveSubsystem::getActualPose,
                    simPipelines,
                    new VisionIOPhotonSim(
                        VisionConstants.CAMERA_0_NAME,
                        VisionConstants.TRANSFORM_TO_CAMERA_0,
                        VisionConstants.CAMERA_0_PROPERTIES,
                        swerveSubsystem::getActualPose,
                        simPipelines
                    )
                );

                questNavSubsystem = new QuestNavSubsystem(
                    swerveSubsystem::addVisionMeasurement,
                    new QuestNavIOSim(swerveSubsystem)
                );

                // Create a simulated claw
                clawSubsystem = new ClawSim();

                elevatorSubsystem = new Elevator(new ElevatorIOSim());

                // TODO: Add behavior chooser
                // Create an opponent robot simulation
                // OpponentRobotSim opponentRobotSim1 = new OpponentRobotSim(
                //     new Pose2d(10, 2, new Rotation2d()),
                //     OpponentRobotBehavior.FOLLOW_PATH
                // );

                // Create another joystick drive for the opponent robot
                // Controls.Drive opponentRobotDriveControls = new Controls.JoystickDrive(new Joystick(1));

                // Set the default command for the opponent robot
                // opponentRobotSim1.setDefaultCommand(
                //         new TeleopSwerve(opponentRobotSim1, opponentRobotDriveControls, false));
                // opponentRobotSim1.setDefaultCommand(
                //     opponentRobotSim1.opponentRobotPathfindToPoseSupplier(swerveSubsystem::getActualPose)
                // );

                // OpponentRobotSim opponentRobotSim2 = new OpponentRobotSim(
                //     new Pose2d(10, 2, new Rotation2d()),
                //     OpponentRobotBehavior.FOLLOW_PATH
                // );
                // opponentRobotSim2.setDefaultCommand(
                //     opponentRobotSim2.opponentRobotPathfindToPoseSupplier(swerveSubsystem::getActualPose)
                // );

                // TODO: refactor
                // opponentRobotDriveControls
                //         .getJoystickButtonOf(opponentRobotDriveControls.zeroGyroButton)
                //         .onTrue(new InstantCommand(() -> opponentRobotSim1.zeroGyro()));
                break;
        }

        // Set up auto routines
        // autoRoutines = new AutoRoutines(swerveSubsystem, elevatorSubsystem, clawSubsystem, visionSubsystem);
        autoRoutines = new AutoRoutines(swerveSubsystem, elevatorSubsystem, clawSubsystem);

        // Set up teleop swerve command
        new TeleopSwerve(
            swerveSubsystem,
            // Switch between joystick and main drive controls depending on the mode
            Controls.isUsingJoystickDrive ? Controls.joystickDriveControls : Controls.mainDriveControls,
            true
        );

        // Register named commands
        NamedCommands.registerCommand(
            "ResetSuperstructure",
            autoRoutines.setUpSuperstructure(SuperstructureConstants.Level.INITIAL_POSITION)
        );

        NamedCommands.registerCommand(
            "SetupSuperstructureL1Auto",
            autoRoutines.setUpSuperstructure(SuperstructureConstants.Level.L1_AUTO)
        );
        NamedCommands.registerCommand(
            "SetupSuperstructureL2",
            autoRoutines.setUpSuperstructure(SuperstructureConstants.Level.L2)
        );
        NamedCommands.registerCommand(
            "SetupSuperstructureL3",
            autoRoutines.setUpSuperstructure(SuperstructureConstants.Level.L3)
        );
        NamedCommands.registerCommand(
            "SetupSuperstructureL4",
            autoRoutines.setUpSuperstructure(SuperstructureConstants.Level.L4)
        );

        NamedCommands.registerCommand("ScoreL4", autoRoutines.doClawMovementsForL4());
        NamedCommands.registerCommand("ScoreCoral", clawSubsystem.runOuttakeUntilCoralIsNotInClaw());

        NamedCommands.registerCommand("IntakeCoral", clawSubsystem.runIntakeUntilCoralIsInClaw());

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        if (Constants.enableSysId) {
            setupSysIdRoutines();
        }

        // TODO: Temporary
        autoChooser.addOption("Coral and Go To All Reefs Test", autoRoutines.getCoralAndGoToAllReefsTest());

        autoChooser.addDefaultOption("Actually move forward", autoRoutines.actuallyMoveForward());
        autoChooser.addOption("Push another robot forward", autoRoutines.pushAnotherRobotForward());

        autoChooser.addOption("runIntakeUntilCoralIsInClaw", clawSubsystem.runIntakeUntilCoralIsInClaw());
        autoChooser.addOption("runOuttakeUntilCoralIsNotInClaw", clawSubsystem.runOuttakeUntilCoralIsNotInClaw());

        // Command to refresh the config
        SmartDashboard.putData("Refresh Tunable Config", TunableValue.refreshConfig);

        // Configure the button bindings
        ButtonBindings buttonBindings = new ButtonBindings(autoRoutines);
        buttonBindings.configureButtonBindings();
        buttonBindings.assignDefaultCommands();

        if (Constants.currentMode == Constants.Mode.SIM) {
            // Sim button bindings
            buttonBindings.configureSimulationBindings();
        }
    }

    /**
     * Adds SysId routines to the auto chooser.
     */
    private void setupSysIdRoutines() {
        // Simple characterization routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            SwerveSysidRoutines.wheelRadiusCharacterization(swerveSubsystem)
        );
        autoChooser.addOption(
            "Drive Simple FF Characterization",
            SwerveSysidRoutines.feedforwardCharacterization(swerveSubsystem)
        );
        autoChooser.addOption(
            "Drive Wheel Slip Current Characterization",
            SwerveSysidRoutines.wheelSlipCurrentCharacterization(swerveSubsystem)
        );
        autoChooser.addOption(
            "Max Acceleration and Velocity Test",
            swerveSubsystem.runMaxAccelerationMaxVelocityTest()
        );

        autoChooser.addOption(
            "QuestNav Transform Measure",
            questNavSubsystem.getMeasureTransformCommand(swerveSubsystem, Seconds.of(0.2))
        );

        // Actual SysId routines
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

        autoChooser.addOption(
            "Drive SysId (All 4)",
            new SequentialCommandGroup(
                swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1),
                swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(1),
                swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1),
                swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse)
            )
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous
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
        Logger.recordOutput(
            "ComponentPositions/CoralInClaw",
            clawSubsystem.getCoralInClawPosition(swerveSubsystem, elevatorSubsystem)
        );
    }
}
