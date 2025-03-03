package frc.robot.subsystems.superstructure.claw;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * The subsystem responsible for the arm.
 */
public class Claw extends SubsystemBase {

    /**
     * The IO interface for the claw.
     */
    protected final ClawIO io;

    /**
     * The inputs for the claw.
     */
    protected final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

    /**
     * Creates a new Claw subsystem.
     * @param io - The IO interface for the claw.
     */
    public Claw(ClawIO io) {
        this.io = io;
    }

    /**
     * @return A command to run the claw.
     * @param motorInputSupplier - the supplier for the percent motor input.
     */
    public Command getRunCommand(DoubleSupplier motorInputSupplier) {
        return new RunCommand(() -> io.runOutake(motorInputSupplier.getAsDouble()), this);
    }

    /**
     * @return A command to move the claw to a specific angle.
     * @param rotationToRotateClawTo - the angle to move to
     */
    public Command getAngleCommand(Rotation2d rotationToRotateClawTo) {
        return new InstantCommand(() -> io.setTurnPosition(rotationToRotateClawTo), this);
    }

    @Override
    public void periodic() {
        io.periodic();

        // Update inputs
        io.updateInputs(inputs);
        Logger.processInputs("Claw", inputs);
    }

    /**
     * @return The position of the claw.
     */
    // @AutoLogOutput(key = "ComponentPositions/Claw")
    public Pose3d getPose(Pose3d elevatorPose) {
        return new Pose3d(
            ClawConstants.RotationPositions.ELEVATOR_TO_CLAW_X,
            0,
            ClawConstants.RotationPositions.ELEVATOR_TO_CLAW_Z + elevatorPose.getZ(),
            new Rotation3d(0, inputs.turnPositionRad, 0)
        );
    }

    public Pose3d getPose() {
        return getPose(new Pose3d());
    }
    /**
     * @return The pose of the coral in the claw.
     * If the claw is not holding coral, return an empty array.
     */
    // public Pose3d[] getCoralInClawPosition(SwerveDrive swerveSubsystem, Elevator elevatorSubsystem) {
    //     // If the claw is not holding coral, return an empty array
    //     if (!inputs.isCoralInClaw) {
    //         return new Pose3d[] {};
    //     }

    //     Translation2d twoDPosition = swerveSubsystem.getActualPose().getTranslation().plus(
    //         getPose(elevatorSubsystem.getStage2Pose())
    //             .getTranslation()
    //             .toTranslation2d()
    //             .plus(ClawConstants.RotationPositions.getClawToCoralX(inputs.turnPositionRad)).rotateBy(swerveSubsystem.getActualPose().getRotation()));

    //     return new Pose3d[] {
    //         new Pose3d(

    //             new Rotation3d()
    //         )
    //     }
    // }
}
