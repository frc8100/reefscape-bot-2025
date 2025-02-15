package frc.robot.subsystems.superstructure.claw;

import static frc.lib.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

// TODO: add generic extensible
/**
 * IO implementation for spark motors
 */
public class ClawIOSpark implements ClawIO {
    private static final double reduction = 1.0;
    private static final boolean inverted = false;
    private int currentLimit = 40;

    /**
     * The motor for the arm.
     */
    private final SparkMax clawMotor;

    private final RelativeEncoder encoder;
    private final SparkMaxConfig config;

    private final Debouncer connectedDebouncer = new Debouncer(0.5);
    private boolean brakeModeEnabled = true;

    public ClawIOSpark() {
        // Create the motor and configure it
        clawMotor = new SparkMax(Constants.Arm.clawMotorId, MotorType.kBrushless);
        encoder = clawMotor.getEncoder();

        // Config
        config = new SparkMaxConfig();
        config.idleMode(brakeModeEnabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
                .inverted(inverted)
                .smartCurrentLimit(currentLimit);
        config.encoder
                .positionConversionFactor(reduction)
                .velocityConversionFactor(reduction)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        config.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                clawMotor,
                5,
                () -> clawMotor.configure(
                        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters));
        tryUntilOk(clawMotor, 5, () -> encoder.setPosition(0.0));
    }

    @Override
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

    @Override
    public void stop() {
        clawMotor.stopMotor();
    }

    /**
     * @return A command to run the claw.
     * @param motorInputSupplier - the supplier for the percent motor input.
     */
    // public Command getRunCommand(DoubleSupplier motorInputSupplier) {
    //     return new RunCommand(() -> runClaw(motorInputSupplier.getAsDouble()));
    // }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(clawMotor, encoder::getPosition, position -> inputs.positionRad = position);
        ifOk(clawMotor, encoder::getVelocity, velocity -> inputs.velocityRadPerSec = velocity);
        ifOk(
                clawMotor,
                new DoubleSupplier[] {clawMotor::getBusVoltage, clawMotor::getAppliedOutput},
                x -> inputs.appliedVolts = x[0] * x[1]);
        ifOk(clawMotor, clawMotor::getOutputCurrent, current -> inputs.torqueCurrentAmps = current);
        inputs.connected = connectedDebouncer.calculate(!sparkStickyFault);
    }
}
