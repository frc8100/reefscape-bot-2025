package frc.robot.subsystems.questnav;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision.VisionConsumer;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.ArrayList;
import java.util.List;

/**
 *
 */
public class QuestNavSubsystem extends SubsystemBase {

    /**
     * @return A command that measures the transform between the QuestNav and the robot's coordinate frame.
     * @param swerveSubsystem - The swerve drive subsystem to use for movement.
     * @param timePerMeasurement - The time to wait between measurements.
     */
    public Command getMeasureTransformCommand(SwerveDrive swerveSubsystem, Time timePerMeasurement) {
        ArrayList<String> measuredPoints = new ArrayList<>();

        Debouncer debounce = new Debouncer(timePerMeasurement.in(Seconds), DebounceType.kRising);

        return Commands.runOnce(() -> {
            // Reset poses
            swerveSubsystem.setPose(new Pose2d());
            io.setPose(new Pose2d());
        }).andThen(
            Commands.run(
                () -> {
                    // Spin in place
                    swerveSubsystem.runVelocityChassisSpeeds(new ChassisSpeeds(0, 0, 1));
                    if (!inputs.isTracking) {
                        return;
                    }

                    if (!debounce.calculate(true)) {
                        return;
                    }

                    PoseFrame[] frames = inputs.unreadPoseFrames;

                    for (PoseFrame frame : frames) {
                        Pose2d questPose = frame.questPose();

                        Transform2d measurement = questPose.minus(swerveSubsystem.getPose());

                        measuredPoints.add("(" + measurement.getX() + "," + measurement.getY() + ")");
                    }

                    // Reset debounce
                    debounce.calculate(false);
                },
                swerveSubsystem
            ).finallyDo(() -> {
                System.out.println("Measured points:");
                System.out.println(String.join(",", measuredPoints));
            })
        );
    }

    // TODO: Move these to a constants file
    /**
     * The transform from the robot's center to the headset. Used for offsetting pose data.
     */
    // TODO: Measure and set this transform
    public static final Transform2d ROBOT_TO_QUEST = new Transform2d(0.5, 0.3, new Rotation2d());
    private static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );

    /**
     * The battery percentage threshold to trigger a low battery alert.
     * At or below this percentage, a warning alert will be triggered.
     */
    private static final int LOW_BATTERY_THRESHOLD = 20;

    /**
     * Consumer to send pose data to.
     */
    private final VisionConsumer consumer;

    private final QuestNavIO io;
    private final QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();

    // Alerts
    private final Alert notConnectedAlert = new Alert("QuestNav not connected.", Alert.AlertType.kWarning);
    private final Alert lowBatteryAlert = new Alert("QuestNav battery low.", Alert.AlertType.kWarning);

    /**
     * Creates a new QuestNavSubsystem.
     * @param consumer - The vision consumer to send pose data to.
     * @param io - The IO implementation to use (real or simulated).
     */
    public QuestNavSubsystem(VisionConsumer consumer, QuestNavIO io) {
        this.consumer = consumer;
        this.io = io;
    }

    @Override
    public void periodic() {
        // Update the IO inputs
        io.updateInputs(inputs);

        // Handle alerts
        notConnectedAlert.set(inputs.connected);
        lowBatteryAlert.set(inputs.batteryPercent >= 0 && inputs.batteryPercent <= LOW_BATTERY_THRESHOLD);

        if (!inputs.isTracking) {
            // Not currently tracking, so don't do anything
            return;
        }

        // Get the latest pose data frames from the Quest
        PoseFrame[] questFrames = inputs.unreadPoseFrames;

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Get the pose of the Quest
            Pose2d questPose = questFrame.questPose();
            // Get timestamp for when the data was sent
            double timestamp = questFrame.dataTimestamp();

            // Transform by the mount pose to get your robot pose
            Pose2d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

            // You can put some sort of filtering here if you would like!

            // Add the measurement to our estimator
            consumer.accept(robotPose, timestamp, QUESTNAV_STD_DEVS);
        }
    }
}
