package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * The subsystem responsible for the arm.
 */
public class ArmSubsystem extends SubsystemBase {
    /**
     * The motor for the arm.
     */
    private SparkMax clawMotor;

    public ArmSubsystem() {
        // Create the motor and configure it
        clawMotor = new SparkMax(Constants.Arm.clawMotorId, MotorType.kBrushless);
    }

    /**
     * Runs the claw motor.
     * @param motorInput - percent input, from [-1, 1] without deadband.
     */
    public void runClaw(double motorInput) {
        // Apply deadband
        motorInput = MathUtil.applyDeadband(motorInput, Constants.Arm.armDeadband);

        // Run the motor
        double percentOutput = Constants.Arm.armPercentOutput * motorInput;
        clawMotor.set(percentOutput);

        // Log
        Logger.recordOutput("Arm/motorInput", motorInput);
        Logger.recordOutput("Arm/percentOutput", percentOutput);
    }

    /**
     * @return A command to run the claw.
     * @param motorInputSupplier - the supplier for the percent motor input.
     */
    // public Command getRunCommand(DoubleSupplier motorInputSupplier) {
    //     return new RunCommand(() -> runClaw(motorInputSupplier.getAsDouble()));
    // }
}
