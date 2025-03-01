package frc.robot.subsystems.superstructure.claw;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * The subsystem responsible for the arm.
 */
public class Claw extends SubsystemBase {
    private final ClawIO io;
    private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

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
        return new RunCommand(() -> io.setTurnPosition(rotationToRotateClawTo), this);
    }

    @Override
    public void periodic() {
        io.periodic();

        // Update inputs
        io.updateInputs(inputs);
        Logger.processInputs("Claw", inputs);
    }
}
