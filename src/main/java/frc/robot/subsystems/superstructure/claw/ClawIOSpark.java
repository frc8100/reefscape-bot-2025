package frc.robot.subsystems.superstructure.claw;

import static frc.lib.util.SparkUtil.*;
import frc.lib.util.GenericSparkIO;

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

/**
 * IO implementation for spark motors
 */
public class ClawIOSpark implements ClawIO {
    /**
     * The motor for the claw.
     */
    private final SparkMax clawMotor;

    /**
     * The relative encoder for the claw.
     * Determines the position and velocity of the claw.
     */
    private final RelativeEncoder encoder;
    private final SparkMaxConfig config;

    /**
     * A debouncer for the connected state, to prevent flickering.
     */
    private final Debouncer connectedDebouncer = new Debouncer(0.5);

    /**
     * The default config for the spark motor.
     */
    private static class DefaultConfig extends GenericSparkIOConfig {
        public DefaultConfig() {
            super();

            // Override the default config
            this.idleMode = SparkBaseConfig.IdleMode.kBrake;
            this.inverted = Constants.Claw.inverted;
            this.smartCurrentLimit = Constants.Claw.currentLimit;
            this.gearRatio = Constants.Claw.reduction;
        }
    }

    public ClawIOSpark() {
        // Create the motor and configure it
        clawMotor = new SparkMax(Constants.Claw.clawMotorId, MotorType.kBrushless);
        encoder = clawMotor.getEncoder();

        config = GenericSparkIO.getDefaultConfig(new DefaultConfig());

        // Apply the config
        tryUntilOk(
                clawMotor,
                5,
                () -> clawMotor.configure(
                        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters));

        // Reset the encoder
        tryUntilOk(clawMotor, 5, () -> encoder.setPosition(0.0));
    }

    @Override
    public void runMotor(double motorInput) {
        // Apply deadband
        motorInput = MathUtil.applyDeadband(motorInput, Constants.Claw.armDeadband);

        // Run the motor
        double percentOutput = Constants.Claw.armPercentOutput * motorInput;
        clawMotor.set(percentOutput);

        // Log
        Logger.recordOutput("Claw/motorInput", motorInput);
        Logger.recordOutput("Claw/percentOutput", percentOutput);
    }

    @Override
    public void stop() {
        clawMotor.stopMotor();
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        // Reset spark sticky fault
        sparkStickyFault = false;

        // Set the position and velocity
        ifOk(clawMotor, encoder::getPosition, position -> inputs.positionRad = position);
        ifOk(clawMotor, encoder::getVelocity, velocity -> inputs.velocityRadPerSec = velocity);

        // Set the supply current based on the bus voltage multiplied by the applied output
        ifOk(
                clawMotor,
                new DoubleSupplier[] { clawMotor::getBusVoltage, clawMotor::getAppliedOutput },
                x -> inputs.appliedVolts = x[0] * x[1]);

        // Set the torque current
        ifOk(clawMotor, clawMotor::getOutputCurrent, current -> inputs.torqueCurrentAmps = current);

        // Set the connected state with a debouncer
        inputs.connected = connectedDebouncer.calculate(!sparkStickyFault);

        // Set the temperature
        ifOk(clawMotor, clawMotor::getMotorTemperature, temp -> inputs.tempCelsius = temp);
    }
}
