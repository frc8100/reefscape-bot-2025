package frc.robot.subsystems.swerve;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/**
 * Simulates another robot.
 * This simulation is not as detailed/complex as the main robot simulation.
 */
public class OpponentRobotSim extends SubsystemBase implements SwerveDrive {

    /**
     * Determines the behavior of opponent robots.
     * ! Note: this does not automatically set any commands.
     */
    public static enum OpponentRobotBehavior {
        /**
         * Follows a path.
         * The default behavior.
         */
        FollowPath,

        /**
         * Use teleop swerve.
         * Enables more resource-intensive
         */
        TeleopSwerve,
    }

    /** List of opponent robot poses */
    private static ArrayList<Supplier<Pose2d>> opponentRobotPoses = new ArrayList<>();

    /** @return The list of opponent robot poses */
    // @AutoLogOutput(key = "Odometry/OpponentRobotPoses")
    public static Pose2d[] getOpponentRobotPoses() {
        return opponentRobotPoses.stream().map(Supplier::get).toArray(Pose2d[]::new);
    }

    /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d()),
    };

    public OpponentRobotBehavior behavior = OpponentRobotBehavior.FollowPath;

    /** PathPlanner configuration */
    // private static final RobotConfig PP_CONFIG = new RobotConfig(
    //         55, // Robot mass in kg
    //         8,  // Robot MOI
    //         new ModuleConfig(
    //                 Units.inchesToMeters(2), 3.5, 1.2, DCMotor.getFalcon500(1).withReduction(8.14), 60, 1), // Swerve
    // module config
    //         0.6 // Track length and width
    // );
    private static final RobotConfig PP_CONFIG = SwerveConfig.getRobotConfig();

    private static final PathConstraints PATH_CONSTRAINTS = SwerveConfig.pathConstraints;

    /** PathPlanner PID settings */
    private final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.02),
        new PIDConstants(7.0, 0.05)
    );

    private final SelfControlledSwerveDriveSimulation simulatedDrive;

    // private Pose2d queeningPose;
    // private int id;

    public OpponentRobotSim(Pose2d startingPose, OpponentRobotBehavior behavior) {
        this.behavior = behavior;

        // Set the queening pose based on the ID
        // try {
        //     this.id = id;
        //     this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
        // } catch (ArrayIndexOutOfBoundsException e) {
        //     this.id = 0;
        //     this.queeningPose = ROBOT_QUEENING_POSITIONS[0];
        //     System.out.println("OpponentRobotSim: Invalid ID " + id + ", defaulting to 0");
        // }

        // Create the SelfControlledSwerveDriveSimulation instance
        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(SwerveConfig.mapleSimConfig, startingPose)
        );

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        // Add the pose supplier to the list
        opponentRobotPoses.add(this::getActualPose);
    }

    /**
     * @return A command to follow a path.
     */
    public Command opponentRobotFollowPath(PathPlannerPath path) {
        return new FollowPathCommand(
            path,
            // Provide actual robot pose in simulation, bypassing odometry error
            simulatedDrive::getActualPoseInSimulationWorld,
            // Provide actual robot speed in simulation, bypassing encoder measurement error
            simulatedDrive::getActualSpeedsRobotRelative,
            // Chassis speeds output
            (speeds, feedforwards) -> simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), false, false),
            driveController,
            PP_CONFIG,
            // Flip path based on alliance side
            () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red),
            this
        );
    }

    /**
     * @return A command to pathfind to a pose.
     */
    public Command opponentRobotPathfindToPose(Pose2d targetPose) {
        return new PathfindingCommand(
            targetPose,
            PATH_CONSTRAINTS,
            simulatedDrive::getActualPoseInSimulationWorld,
            simulatedDrive::getActualSpeedsRobotRelative,
            (speeds, feedforwards) -> simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), false, false),
            driveController,
            PP_CONFIG,
            this
        );
    }

    /**
     * @return A command to recursively pathfind to a pose.
     */
    public Command opponentRobotPathfindToPoseSupplier(Supplier<Pose2d> poseSupplier) {
        return new PathfindingCommand(
            poseSupplier.get(),
            PATH_CONSTRAINTS,
            simulatedDrive::getActualPoseInSimulationWorld,
            simulatedDrive::getActualSpeedsRobotRelative,
            (speeds, feedforwards) -> simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), false, false),
            driveController,
            PP_CONFIG,
            this
        )
            // Create a new command to recursively pathfind to the next pose every 1 second
            .withTimeout(1)
            .finallyDo(() -> {
                // System.out.println("OpponentRobotSim: Pathfinding complete");
                Command nextCommand = opponentRobotPathfindToPoseSupplier(poseSupplier);
                // nextCommand.schedule();
                setDefaultCommand(nextCommand);
            });
    }

    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        this.simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), true, true);
    }

    @Override
    public void runVelocityChassisSpeeds(ChassisSpeeds speed) {
        this.simulatedDrive.runChassisSpeeds(speed, new Translation2d(), true, true);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        simulatedDrive.runSwerveStates(desiredStates);
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        return simulatedDrive.getMeasuredStates();
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return simulatedDrive.getLatestModulePositions();
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        switch (behavior) {
            default:
            case FollowPath:
                // Return actual pose to save resources
                return simulatedDrive.getActualSpeedsFieldRelative();
            case TeleopSwerve:
                // Return accurate odometry pose
                return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
        }
    }

    @Override
    public Rotation2d getGyroHeading() {
        // return simulatedDrive.getRawGyroAngle();
        // Returns the gyro angle in the simulation world to save resources
        return getPose().getRotation();
    }

    @Override
    public Pose2d getPose() {
        switch (behavior) {
            default:
            case FollowPath:
                // Return actual pose to save resources
                return simulatedDrive.getActualPoseInSimulationWorld();
            case TeleopSwerve:
                // Return accurate odometry pose
                return simulatedDrive.getOdometryEstimatedPose();
        }
    }

    @Override
    public Pose2d getActualPose() {
        return simulatedDrive.getActualPoseInSimulationWorld();
    }

    @Override
    public void setPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }

    @Override
    public void zeroGyro(double deg) {
        // Set the pose but with a new rotation value
        setPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(deg)));
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        // Unimplemented
        simulatedDrive.addVisionEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        // Only update if teleop swerve is on
        if (behavior == OpponentRobotBehavior.TeleopSwerve) {
            simulatedDrive.periodic();
        }
    }
}
