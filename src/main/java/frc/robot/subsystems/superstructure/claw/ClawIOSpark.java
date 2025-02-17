package frc.robot.subsystems.superstructure.claw;

import static frc.lib.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.GenericSparkIO;
import frc.robot.Constants;

/**
 * IO implementation for spark motors
 */
public class ClawIOSpark implements ClawIO {
    // Turn motor variables
    /**
     * The motor for the rotation of the claw.
     */
    private final SparkMax angleMotor;

    /**
     * The relative encoder for the rotation motor.
     */
    private final RelativeEncoder angleEncoder;

    /**
     * The closed loop controller for the rotation motor.
     */
    private final SparkClosedLoopController angleClosedLoopController;

    /**
     * A debouncer for the connected state, to prevent flickering.
     */
    private final Debouncer angleConnectedDebouncer = new Debouncer(0.5);

    // Outtake motor variables
    /**
     * The motor for the outtake of the claw.
     */
    private final SparkMax outakeMotor;

    /**
     * The relative encoder for the outtake motor.
     */ 
    private final RelativeEncoder outakeEncoder;
    /**
     * The closed loop controller for the outtake motor.
     */
    // private final SparkClosedLoopController outakeClosedLoopController;
    
    /**
     * A debouncer for the connected state, to prevent flickering.
     */
    private final Debouncer outakeConnectedDebouncer = new Debouncer(0.5);

    /**
     * The config for the turn motor.
     */
    private static class AngleConfig extends GenericSparkIOConfig {
        public AngleConfig() {
            super();

            // Override the default config
            this.idleMode = SparkBaseConfig.IdleMode.kBrake;
            this.inverted = Constants.Claw.angleMotorInverted;
            this.smartCurrentLimit = Constants.Claw.angleMotorCurrentLimit;
            this.gearRatio = Constants.Claw.angleGearRatio;
        }
    }

    /**
     * The config for the outtake motor.
     */
    private static class OutakeConfig extends GenericSparkIOConfig {
        public OutakeConfig() {
            super();

            // Override the default config
            this.idleMode = SparkBaseConfig.IdleMode.kBrake;
            this.inverted = Constants.Claw.outakeMotorInverted;
            this.smartCurrentLimit = Constants.Claw.outakeMotorCurrentLimit;
            this.gearRatio = Constants.Claw.outakeGearRatio;
        }
    }
    public ClawIOSpark() {
        // Create the motor and configure it
        angleMotor = new SparkMax(Constants.Claw.angleClawMotorId, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        angleClosedLoopController = angleMotor.getClosedLoopController();

        SparkMaxConfig angleConfig = new AngleConfig().getConfig();

        // Apply PID config for the angle motor
        angleConfig
                .closedLoop
                .pidf(
                        Constants.Claw.angleKP,
                        Constants.Claw.angleKI,
                        Constants.Claw.angleKD,
                        Constants.Claw.angleKF)
                .outputRange(-Constants.Claw.anglePower, Constants.Claw.anglePower);

        // Apply the config
        tryUntilOk(
                angleMotor,
                5,
                () -> angleMotor.configure(
                        angleConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters));

        // Reset the encoder
        tryUntilOk(angleMotor, 5, () -> angleEncoder.setPosition(0.0));

        // Create the outake motor and configure it
        outakeMotor = new SparkMax(Constants.Claw.outakeClawMotorId, MotorType.kBrushless);
        outakeEncoder = outakeMotor.getEncoder();
        // outakeClosedLoopController = outakeMotor.getClosedLoopController();

        SparkMaxConfig outakeConfig = new OutakeConfig().getConfig();

        // Apply the config
        tryUntilOk(
                outakeMotor,
                5,
                () -> outakeMotor.configure(
                        outakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters));

        // Reset the encoder
        tryUntilOk(outakeMotor, 5, () -> outakeEncoder.setPosition(0.0));
    }

    @Override
    public void runOutake(double motorInput) {
        // Apply deadband
        motorInput = MathUtil.applyDeadband(motorInput, Constants.Claw.armDeadband);

        // Run the motor
        double percentOutput = Constants.Claw.armPercentOutput * motorInput;
        outakeMotor.set(percentOutput);

        // Log
        Logger.recordOutput("Claw/motorInput", motorInput);
        Logger.recordOutput("Claw/percentOutput", percentOutput);
    }

    @Override
    public void stop() {
        angleMotor.stopMotor();
        outakeMotor.stopMotor();
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        // Set the position of the turn motor
        angleClosedLoopController.setReference(rotation.getDegrees(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        // Reset spark sticky fault
        sparkStickyFault = false;

        // Update inputs for the turn motor

        // Set the position and velocity
        ifOk(angleMotor, angleEncoder::getPosition, position -> inputs.turnPositionRad = position);
        ifOk(angleMotor, angleEncoder::getVelocity, velocity -> inputs.turnVelocityRadPerSec = velocity);

        // Set the supply current based on the bus voltage multiplied by the applied output
        ifOk(
                angleMotor,
                new DoubleSupplier[] { angleMotor::getBusVoltage, angleMotor::getAppliedOutput },
                x -> inputs.turnAppliedVolts = x[0] * x[1]);

        // Set the torque current
        ifOk(angleMotor, angleMotor::getOutputCurrent, current -> inputs.turnTorqueCurrentAmps = current);

        // Set the connected state with a debouncer
        inputs.turnConnected = angleConnectedDebouncer.calculate(!sparkStickyFault);

        // Set the temperature
        ifOk(angleMotor, angleMotor::getMotorTemperature, temp -> inputs.turnTempCelsius = temp);

        // Update inputs for the outake motor

        // Set the position and velocity
        ifOk(outakeMotor, outakeEncoder::getPosition, position -> inputs.outakePositionRad = position);
        ifOk(outakeMotor, outakeEncoder::getVelocity, velocity -> inputs.outakeVelocityRadPerSec = velocity);

        // Set the supply current based on the bus voltage multiplied by the applied output
        ifOk(
                outakeMotor,
                new DoubleSupplier[] { outakeMotor::getBusVoltage, outakeMotor::getAppliedOutput },
                x -> inputs.outakeAppliedVolts = x[0] * x[1]);

        // Set the torque current
        ifOk(outakeMotor, outakeMotor::getOutputCurrent, current -> inputs.outakeTorqueCurrentAmps = current);

        // Set the connected state with a debouncer
        inputs.outakeConnected = outakeConnectedDebouncer.calculate(!sparkStickyFault);
    }
}
