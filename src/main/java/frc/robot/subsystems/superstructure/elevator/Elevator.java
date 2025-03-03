package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The elevator subsystem. This subsystem is responsible for controlling the elevator.
 */
public class Elevator extends SubsystemBase {

    /**
     * The IO interface for the elevator.
     */
    protected final ElevatorIO io;

    /**
     * The inputs for the elevator.
     */
    protected final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    /**
     * Creates a new Elevator subsystem.
     * @param io - The IO interface for the elevator.
     */
    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.periodic();

        // Update inputs
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    /**
     * Stops the elevator.
     */
    public void stop() {
        io.stop();
    }

    /**
     * Sets the desired elevator position.
     * @param position - The desired elevator position.
     */
    public void setPosition(Distance position) {
        io.setPosition(position);
    }

    /**
     * @return A command to set the elevator to a certain position.
     */
    public Command getPositionCommand(Distance position) {
        return new InstantCommand(() -> setPosition(position), this);
    }

    /**
     * @return The current position of the 1st stage of the elevator.
     */
    @AutoLogOutput(key = "ComponentPositions/Elevator/Stage1")
    public Pose3d getStage1Pose() {
        return new Pose3d(
            0.0,
            0.0,
            MathUtil.clamp(
                inputs.height - ElevatorConstants.Position.STAGE_1_HEIGHT.in(Meters),
                0,
                ElevatorConstants.Position.STAGE_1_MAX_HEIGHT.in(Meters)
            ),
            new Rotation3d()
        );
    }

    /**
     * @return The current position of the 2nd stage of the elevator.
     */
    @AutoLogOutput(key = "ComponentPositions/Elevator/Stage2")
    public Pose3d getStage2Pose() {
        // TODO
        return new Pose3d(0.0, 0.0, inputs.height, new Rotation3d());
    }
}
